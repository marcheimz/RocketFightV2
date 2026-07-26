#include "server/Sandbox.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

// Declared by <unistd.h> only under _GNU_SOURCE on some libcs, and the child
// needs it whenever the caller asked to inherit rather than replace.
extern char** environ;

namespace rf {
namespace {

using Clock = std::chrono::steady_clock;

// ---------------------------------------------------------------------------
// Everything in this block runs in the forked child, between fork and exec.
//
// The parent is a multi-threaded web server, so the child inherits exactly one
// thread and a heap that any of the others might have been holding a lock on.
// That makes malloc, std::string, iostreams and anything that could touch a
// lock undefined here. Only async-signal-safe calls -- write, open, close,
// setrlimit, execv -- and only stack buffers. Every string the child needs was
// built before the fork.
// ---------------------------------------------------------------------------

void childWrite(int fd, const char* text) {
    const std::size_t n = std::strlen(text);
    // The return value is deliberately ignored: this is a diagnostic on a pipe
    // that may already be gone, and there is nothing useful to do about it.
    ssize_t written = ::write(fd, text, n);
    (void)written;
}

[[noreturn]] void childFail(const char* what) {
    childWrite(STDERR_FILENO, "[sandbox] child setup failed: ");
    childWrite(STDERR_FILENO, what);
    childWrite(STDERR_FILENO, "\n");
    _exit(126);
}

void setLimit(int which, std::uint64_t soft, std::uint64_t hard) {
    struct rlimit rl;
    rl.rlim_cur = static_cast<rlim_t>(soft);
    rl.rlim_max = static_cast<rlim_t>(hard);
    // Best effort. A limit we cannot lower is a limit that was already lower,
    // or a kernel that disagrees; neither is worth refusing to run over.
    (void)::setrlimit(which, &rl);
}

// Writes one small string to a /proc file. Used for the uid/gid maps, which is
// the part that keeps the child *itself* rather than turning it into `nobody`.
bool writeProcFile(const char* path, const char* text) {
    const int fd = ::open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return false;
    const std::size_t n  = std::strlen(text);
    const ssize_t     rc = ::write(fd, text, n);
    ::close(fd);
    return rc == static_cast<ssize_t>(n);
}

// Network isolation, best effort, in that order for a reason.
//
// An unprivileged process cannot create a bare network namespace, but it can
// create a *user* namespace and get a network namespace along with it -- which
// is how this works at all without the server being root. The identity uid/gid
// map afterwards is not optional: without it the child lands as the overflow
// uid and loses the ability to read its own working directory, which looks
// exactly like a broken submission.
bool tryIsolateNetwork(const char* uidMap, const char* gidMap) {
    if (::unshare(CLONE_NEWUSER | CLONE_NEWNET) == 0) {
        // setgroups must be denied before gid_map may be written; a kernel that
        // predates the file simply has nothing to deny.
        (void)writeProcFile("/proc/self/setgroups", "deny");
        const bool mapped = writeProcFile("/proc/self/gid_map", gidMap) &&
                            writeProcFile("/proc/self/uid_map", uidMap);
        if (!mapped) {
            childWrite(STDERR_FILENO,
                       "[sandbox] warning: uid map failed; running as the overflow uid\n");
        }
        return true;
    }
    // The privileged path, for a host that does have CAP_SYS_ADMIN.
    return ::unshare(CLONE_NEWNET) == 0;
}

// ---------------------------------------------------------------------------
// Parent-side helpers.
// ---------------------------------------------------------------------------

bool drain(int fd, std::string& into, std::uint64_t cap, bool& truncated) {
    char buf[16384];
    for (;;) {
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0) {
            if (into.size() < cap) {
                const std::size_t room = cap - into.size();
                const auto        take = static_cast<std::size_t>(n);
                into.append(buf, take < room ? take : room);
                if (take > room) truncated = true;
            } else {
                truncated = true;
            }
            continue;
        }
        if (n == 0) return false;                        // EOF
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return true;  // nothing more for now
        return false;
    }
}

std::string decimal(std::uint64_t v) { return std::to_string(v); }

}  // namespace

std::string SandboxResult::describe() const {
    switch (outcome) {
        case SandboxOutcome::Ok:
            return "ok";
        case SandboxOutcome::NonZeroExit:
            return "exited " + std::to_string(exitCode);
        case SandboxOutcome::Signalled:
            return std::string("killed by signal ") + std::to_string(signal) + " (" +
                   ::strsignal(signal) + ")";
        case SandboxOutcome::TimedOut:
            if (!diagnostic.empty()) return "SIGKILLed: " + diagnostic;
            return "timed out after " + std::to_string(wallSeconds) + "s and was SIGKILLed";
        case SandboxOutcome::SpawnFailed:
            break;
    }
    return "failed to start: " + diagnostic;
}

bool networkIsolationAvailable() {
    // Probed by actually doing it, once, in a throwaway child. Asking the
    // kernel whether it would have allowed it means guessing at seccomp
    // policies, container configuration and sysctls; doing it is one fork.
    static const bool available = [] {
        const pid_t pid = ::fork();
        if (pid < 0) return false;
        if (pid == 0) {
            const bool ok = ::unshare(CLONE_NEWUSER | CLONE_NEWNET) == 0 ||
                            ::unshare(CLONE_NEWNET) == 0;
            _exit(ok ? 0 : 1);
        }
        int status = 0;
        while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
        return WIFEXITED(status) && WEXITSTATUS(status) == 0;
    }();
    return available;
}

SandboxResult runSandboxed(const std::vector<std::string>& argv, const std::string& workingDir,
                           const SandboxLimits& limits, const std::vector<std::string>& env) {
    SandboxResult result;

    if (argv.empty()) {
        result.diagnostic = "empty argv";
        return result;
    }

    // Everything the child touches is built here, while allocation is still
    // legal. After the fork it may only read these buffers.
    std::vector<char*> cargv;
    cargv.reserve(argv.size() + 1);
    for (const std::string& a : argv) cargv.push_back(const_cast<char*>(a.c_str()));
    cargv.push_back(nullptr);

    std::vector<char*> cenv;
    cenv.reserve(env.size() + 1);
    for (const std::string& e : env) cenv.push_back(const_cast<char*>(e.c_str()));
    cenv.push_back(nullptr);
    char** childEnv = env.empty() ? ::environ : cenv.data();

    const char* exePath = argv.front().c_str();
    const char* cwd     = workingDir.empty() ? nullptr : workingDir.c_str();

    const uid_t       uid    = ::getuid();
    const gid_t       gid    = ::getgid();
    const std::string uidMap = decimal(uid) + " " + decimal(uid) + " 1";
    const std::string gidMap = decimal(gid) + " " + decimal(gid) + " 1";

    const bool wantNetIsolation = networkIsolationAvailable();

    int outPipe[2];
    int errPipe[2];
    if (::pipe2(outPipe, O_CLOEXEC) != 0) {
        result.diagnostic = std::string("pipe2: ") + std::strerror(errno);
        return result;
    }
    if (::pipe2(errPipe, O_CLOEXEC) != 0) {
        result.diagnostic = std::string("pipe2: ") + std::strerror(errno);
        ::close(outPipe[0]);
        ::close(outPipe[1]);
        return result;
    }

    const auto  started = Clock::now();
    const pid_t pid     = ::fork();
    if (pid < 0) {
        result.diagnostic = std::string("fork: ") + std::strerror(errno);
        ::close(outPipe[0]);
        ::close(outPipe[1]);
        ::close(errPipe[0]);
        ::close(errPipe[1]);
        return result;
    }

    if (pid == 0) {
        // ---- child ---------------------------------------------------------
        // Its own session, so the deadline can be enforced against the whole
        // process group. A submission that forks and then exits would otherwise
        // leave its children running with nothing left to kill them by.
        if (::setsid() < 0) childFail("setsid");

        if (::dup2(outPipe[1], STDOUT_FILENO) < 0) childFail("dup2 stdout");
        if (::dup2(errPipe[1], STDERR_FILENO) < 0) childFail("dup2 stderr");
        // stdin is /dev/null, not the parent's terminal: a submission that
        // reads stdin should see EOF rather than block forever waiting for a
        // human who is not there.
        const int devNull = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
        if (devNull >= 0) {
            (void)::dup2(devNull, STDIN_FILENO);
            ::close(devNull);
        }

        if (cwd != nullptr && ::chdir(cwd) != 0) childFail("chdir to the scratch directory");

        if (wantNetIsolation && !tryIsolateNetwork(uidMap.c_str(), gidMap.c_str())) {
            childWrite(STDERR_FILENO, "[sandbox] warning: network isolation unavailable\n");
        }

        // No setuid binary the child execs can gain privileges, and neither can
        // anything it execs afterwards. One syscall, and it is inherited.
        (void)::prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0);

        // CPU time gets a hard limit one second above the soft one: the soft
        // limit raises SIGXCPU, which a submission could catch and ignore, and
        // the hard limit is the SIGKILL behind it. The wall-clock deadline in
        // the parent is the real backstop for both.
        setLimit(RLIMIT_CPU, limits.cpuSeconds, limits.cpuSeconds + 1);
        setLimit(RLIMIT_AS, limits.addressSpaceBytes, limits.addressSpaceBytes);
        setLimit(RLIMIT_FSIZE, limits.fileSizeBytes, limits.fileSizeBytes);
        setLimit(RLIMIT_NOFILE, limits.openFiles, limits.openFiles);
        // Caps threads as well as processes on Linux, which is intended: the
        // runner is single-threaded and a submission that spawns a hundred of
        // anything is not doing control theory.
        setLimit(RLIMIT_NPROC, limits.processes, limits.processes);
        // No core dumps. A crash inside a submission is a normal, expected,
        // frequently-occurring result, and each one would otherwise write a
        // multi-hundred-megabyte image of the runner's address space.
        setLimit(RLIMIT_CORE, 0, 0);

        ::execve(exePath, cargv.data(), childEnv);
        childFail("execve");
    }

    // ---- parent ------------------------------------------------------------
    ::close(outPipe[1]);
    ::close(errPipe[1]);

    // Non-blocking, because the poll loop below has a deadline to keep and a
    // blocking read on a pipe a hung child is still holding never returns.
    (void)::fcntl(outPipe[0], F_SETFL, O_NONBLOCK);
    (void)::fcntl(errPipe[0], F_SETFL, O_NONBLOCK);

    const auto deadline =
        started + std::chrono::microseconds(static_cast<long long>(limits.wallSeconds * 1e6));

    bool outOpen   = true;
    bool errOpen   = true;
    bool timedOut  = false;
    bool floodKill = false;
    bool reaped    = false;
    int  status    = 0;

    for (;;) {
        const auto now = Clock::now();
        if (now >= deadline) {
            timedOut = true;
            break;
        }

        if (!outOpen && !errOpen) {
            // Output is finished; all that is left is the exit status. Poll for
            // it rather than blocking, because a child that closed its pipes
            // and then hung is exactly the case the deadline exists for.
            const pid_t r = ::waitpid(pid, &status, WNOHANG);
            if (r == pid) {
                reaped = true;
                break;
            }
            if (r < 0 && errno != EINTR) break;
            struct timespec nap = {0, 500000};  // 0.5 ms
            (void)::nanosleep(&nap, nullptr);
            continue;
        }

        struct pollfd fds[2];
        nfds_t        n = 0;
        if (outOpen) fds[n++] = pollfd{outPipe[0], POLLIN, 0};
        if (errOpen) fds[n++] = pollfd{errPipe[0], POLLIN, 0};

        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
        const int waitMs = static_cast<int>(remaining < 1 ? 1 : (remaining > 50 ? 50 : remaining));

        const int pr = ::poll(fds, n, waitMs);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        for (nfds_t i = 0; i < n; ++i) {
            if (fds[i].revents == 0) continue;
            const bool isOut = fds[i].fd == outPipe[0];
            std::string& sink = isOut ? result.out : result.err;
            const bool still =
                drain(fds[i].fd, sink, limits.maxOutputBytes, result.outputTruncated);
            if (!still) (isOut ? outOpen : errOpen) = false;
        }
        if (result.outputTruncated) {
            // A submission printing without end is a submission being killed:
            // there is no output past the cap worth waiting for.
            floodKill = true;
            break;
        }
    }

    if (!reaped) {
        // SIGKILL, not SIGTERM, and to the whole group. A spinning child can
        // ignore a polite signal; nothing ignores this one, and killing the
        // group catches anything it forked before it hung.
        ::kill(-pid, SIGKILL);
        ::kill(pid, SIGKILL);
        while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    }

    // Whatever the child managed to say before it died is still evidence, so
    // the pipes are drained one last time rather than dropped on the floor.
    if (outOpen) drain(outPipe[0], result.out, limits.maxOutputBytes, result.outputTruncated);
    if (errOpen) drain(errPipe[0], result.err, limits.maxOutputBytes, result.outputTruncated);
    ::close(outPipe[0]);
    ::close(errPipe[0]);

    result.wallSeconds =
        std::chrono::duration<double>(Clock::now() - started).count();

    if (timedOut) {
        result.outcome = SandboxOutcome::TimedOut;
    } else if (floodKill) {
        result.outcome    = SandboxOutcome::TimedOut;
        result.diagnostic = "output exceeded " + decimal(limits.maxOutputBytes) + " bytes";
    } else if (WIFSIGNALED(status)) {
        result.outcome = SandboxOutcome::Signalled;
        result.signal  = WTERMSIG(status);
    } else if (WIFEXITED(status)) {
        result.exitCode = WEXITSTATUS(status);
        result.outcome  = result.exitCode == 0 ? SandboxOutcome::Ok : SandboxOutcome::NonZeroExit;
    } else {
        result.outcome    = SandboxOutcome::SpawnFailed;
        result.diagnostic = "child neither exited nor was signalled";
    }
    return result;
}

}  // namespace rf
