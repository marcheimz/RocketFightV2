#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rf {

// ---------------------------------------------------------------------------
// Running somebody else's code, at arm's length.
//
// Read this before trusting it: THIS IS NOT A SECURITY BOUNDARY. It is
// hardening against buggy and casually hostile code -- an infinite loop, a
// runaway allocation, a stray fopen, a fork bomb, a wild pointer, a submission
// that tries to phone home. A determined attacker with arbitrary native code
// execution defeats it, because arbitrary native code execution is exactly what
// a .so submission is, and the only real answers to that are a container, a VM,
// or a seccomp policy far stricter than anything here.
//
// What it does buy, honestly stated:
//
//   - A wall-clock deadline the child cannot influence, enforced with SIGKILL
//     on the whole process group. Not SIGTERM: a spinning submission that
//     installed a handler, or one stuck in a tight loop with signals blocked,
//     would ignore it, and a supervisor whose kill is advisory is not one.
//   - rlimits applied in the child between fork and exec, so they are in force
//     before a single instruction of the submission runs: CPU time, address
//     space, file size, open files, process count, and a core dump size of zero
//     so a crashing submission does not fill the disk with a snapshot of
//     itself.
//   - A working directory of the caller's choosing, so a submission that does
//     write a file writes it somewhere disposable.
//   - Best-effort network isolation via an unprivileged user namespace plus a
//     network namespace. Where that is not permitted it is skipped with a
//     logged warning rather than failing the run, because a host that cannot
//     unshare is a host that can still rank controllers.
//
// What it explicitly does NOT do: filesystem confinement (the child can read
// anything the invoking user can read), syscall filtering, or any defence
// against a submission that attacks the kernel. Say so out loud rather than
// implying safety that is not there.
// ---------------------------------------------------------------------------

struct SandboxLimits {
    // The one the user cares about. The whole 60-second benchmark is about
    // 20 ms of CPU, so anything in seconds is enormously generous and a
    // submission that hits this was not merely slow.
    double wallSeconds{10.0};

    std::uint64_t cpuSeconds{5};                       // RLIMIT_CPU (soft; hard is +1)
    std::uint64_t addressSpaceBytes{1024ull << 20};    // RLIMIT_AS
    std::uint64_t fileSizeBytes{16ull << 20};          // RLIMIT_FSIZE
    std::uint64_t openFiles{64};                       // RLIMIT_NOFILE
    std::uint64_t processes{64};                       // RLIMIT_NPROC -- also caps threads

    // Pipes are not covered by RLIMIT_FSIZE, so a submission printing in a loop
    // would otherwise fill the parent's memory instead of its own disk.
    std::uint64_t maxOutputBytes{4ull << 20};
};

enum class SandboxOutcome {
    Ok,           // exited 0
    NonZeroExit,  // ran to completion, said no
    Signalled,    // died on a signal it did not ask for -- a crash
    TimedOut,     // outlived the deadline and was killed
    SpawnFailed,  // never got as far as running: bad path, fork failed
};

struct SandboxResult {
    SandboxOutcome outcome{SandboxOutcome::SpawnFailed};
    int            exitCode{-1};
    int            signal{0};
    double         wallSeconds{0.0};
    bool           outputTruncated{false};

    std::string out;
    std::string err;
    std::string diagnostic;  // why it failed to spawn, when that is what happened

    bool ok() const { return outcome == SandboxOutcome::Ok; }

    // A single line fit to put in a leaderboard cell. A crashed child is a
    // *result*, not a server error, so this has to read as one.
    std::string describe() const;
};

// Probed once and logged, rather than warned about per run: the answer depends
// on the kernel and the container this server happens to be inside, and it does
// not change while it is running.
bool networkIsolationAvailable();

// Forks, applies the limits, execs argv[0], supervises. `argv` must be
// non-empty and argv[0] must be a path, not a name to search for. `workingDir`
// must already exist.
//
// `env` is a list of "KEY=VALUE" strings and *replaces* the environment rather
// than adding to it, which is the point: a server's own environment routinely
// holds tokens, keys and paths that have nothing to do with flying a rocket,
// and handing all of it to an untrusted child is a habit worth not having. An
// empty list inherits, for callers who genuinely want that.
//
// Never throws. Every failure mode comes back as a SandboxResult, because the
// caller is a web server and a submission that misbehaves must not be able to
// take a request thread with it.
SandboxResult runSandboxed(const std::vector<std::string>& argv, const std::string& workingDir,
                           const SandboxLimits&            limits,
                           const std::vector<std::string>& env = {});

}  // namespace rf
