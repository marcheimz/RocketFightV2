// ---------------------------------------------------------------------------
// The submission boundary: what the host does when somebody else's code is
// wrong.
//
// Everything here is a *negative* test, and that is the point. The happy path is
// covered by the benchmark tests and by the example submissions the server
// ranks; what has to be asserted separately is that each way of being wrong
// produces a recorded result with a reason rather than a crash, a hang, or a
// plausible-looking score. The failure mode this suite exists to prevent is a
// server that stops answering because a stranger's rf_resolve did not return.
//
// The fixtures are built by CMake from tests/fixtures/badlib.c, one variant per
// defect. Their paths arrive through the generated TestPaths.hpp rather than
// being guessed relative to the working directory.
// ---------------------------------------------------------------------------

#include <doctest/doctest.h>

#include <signal.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "TestRockets.hpp"
#include "control/AbiFlyByWire.hpp"
#include "rocketfight/flybywire_abi.h"
#include "server/Sandbox.hpp"
#include "tests/TestPaths.hpp"

namespace {

std::string fixture(const char* name) { return std::string(rf::kFixtureDir) + "/" + name + ".so"; }

// A plausible mid-flight observation, so a fixture that does read its input has
// something finite to read. Nothing here is asserted against; it exists to make
// the call legitimate.
rf::Observation sampleObservation(const rf::RocketSpec& spec) {
    rf::Observation obs;
    obs.tick          = 1000;
    obs.time          = rf::Real(1.0);
    obs.pos           = rf::Vec2{10.0, -5.0};
    obs.vel           = rf::Vec2{3.0, 1.0};
    obs.angle         = rf::Real(0.4);
    obs.angVel        = rf::Real(-0.1);
    obs.maxAccel      = spec.maxForwardAccel();
    obs.maxAngAccel   = spec.maxAngAccel();
    obs.thrusterCount = spec.count();
    return obs;
}

}  // namespace

// ---------------------------------------------------------------------------
// AbiFlyByWire::load -- refusing, with a reason, before anything runs.
// ---------------------------------------------------------------------------

TEST_CASE("a well-formed submission loads and answers") {
    const rf::RocketSpec& spec = rf::testing::classic();
    std::string           error;

    auto fbw = rf::AbiFlyByWire::load(fixture("ok"), spec, error);
    REQUIRE_MESSAGE(fbw != nullptr, error);
    CHECK(error.empty());

    // The fixture writes nothing, and the host zeroes the output before the
    // call, so "wrote nothing" must come back as "everything off" rather than as
    // stack garbage.
    const rf::ControlInput out = fbw->resolve(rf::Intent{}, sampleObservation(spec));
    for (std::size_t i = 0; i < spec.count(); ++i) {
        CHECK(out.level[i] == doctest::Approx(0.0));
        CHECK(out.gimbal[i] == doctest::Approx(0.0));
    }
    CHECK_FALSE(out.fire);
}

TEST_CASE("a library that cannot be dlopened is refused, not crashed on") {
    const rf::RocketSpec& spec = rf::testing::classic();
    std::string           error;

    SUBCASE("no such file") {
        CHECK(rf::AbiFlyByWire::load(fixture("does-not-exist"), spec, error) == nullptr);
        CHECK_FALSE(error.empty());
        CHECK(error.find("dlopen failed") != std::string::npos);
    }

    SUBCASE("a file that is not an ELF object at all") {
        // The realistic version of this is a submitter uploading their source
        // and calling it a binary. dlopen says so; the host repeats what it
        // said rather than inventing a diagnosis of its own.
        const std::string path = std::string(rf::kFixtureDir) + "/not-an-object.so";
        {
            std::ofstream os(path, std::ios::binary | std::ios::trunc);
            os << "int main(void) { return 0; }\n";
        }
        CHECK(rf::AbiFlyByWire::load(path, spec, error) == nullptr);
        CHECK(error.find("dlopen failed") != std::string::npos);
    }

    SUBCASE("a bare filename is refused before dlopen sees it") {
        // Otherwise dlopen hunts the loader search path and "which library did
        // we just evaluate?" becomes a question about the host's environment.
        CHECK(rf::AbiFlyByWire::load("ok.so", spec, error) == nullptr);
        CHECK(error.find("must contain a '/'") != std::string::npos);
    }
}

TEST_CASE("a missing entry point is a load failure naming the symbol") {
    const rf::RocketSpec& spec = rf::testing::classic();
    std::string           error;

    CHECK(rf::AbiFlyByWire::load(fixture("no_resolve"), spec, error) == nullptr);
    CHECK(error.find("rf_resolve") != std::string::npos);
    // The message has to be actionable for the overwhelmingly most common cause,
    // which is a C++ submission that forgot extern "C".
    CHECK(error.find("extern \"C\"") != std::string::npos);
}

TEST_CASE("an ABI version mismatch is refused before rf_init runs") {
    const rf::RocketSpec& spec = rf::testing::classic();
    std::string           error;

    CHECK(rf::AbiFlyByWire::load(fixture("wrong_version"), spec, error) == nullptr);
    CHECK(error.find("ABI version mismatch") != std::string::npos);
    CHECK(error.find(std::to_string(RF_ABI_VERSION + 1000)) != std::string::npos);
}

TEST_CASE("rf_init declining the vehicle is a refusal, not a crash") {
    const rf::RocketSpec& spec = rf::testing::classic();
    std::string           error;

    CHECK(rf::AbiFlyByWire::load(fixture("init_declines"), spec, error) == nullptr);
    CHECK(error.find("declined") != std::string::npos);
    CHECK(error.find("classic") != std::string::npos);
}

TEST_CASE("non-finite and out-of-range commands are sanitised, not propagated") {
    const rf::RocketSpec& spec = rf::testing::classic();
    std::string           error;

    auto fbw = rf::AbiFlyByWire::load(fixture("nan"), spec, error);
    REQUIRE_MESSAGE(fbw != nullptr, error);

    const rf::ControlInput out = fbw->resolve(rf::Intent{}, sampleObservation(spec));
    for (std::size_t i = 0; i < spec.count(); ++i) {
        // A NaN reaching the physics would poison the world state and therefore
        // the state hash -- which would make a submission's own garbage look
        // like the host being nondeterministic.
        CHECK(std::isfinite(out.level[i]));
        CHECK(std::isfinite(out.gimbal[i]));
        // Replaced by zero, not clamped: clamping a NaN keeps it, and clamping
        // an infinity to the range limit would turn "this controller computed
        // nonsense" into "this controller commanded hard over", which is a
        // command it never meant and one that scores.
        CHECK(out.level[i] == doctest::Approx(0.0));
        CHECK(out.gimbal[i] == doctest::Approx(0.0));
    }
    CHECK(out.fire);  // an int flag has no non-finite state; it is passed through
}

// ---------------------------------------------------------------------------
// The supervisor. These are the tests that justify the process boundary.
// ---------------------------------------------------------------------------

TEST_CASE("a child that blocks forever is killed at the wall-clock deadline") {
    // Sleeping rather than spinning, deliberately: this consumes no CPU time and
    // no memory, so RLIMIT_CPU and RLIMIT_AS never fire. If the deadline is
    // broken, this test hangs the suite -- which is exactly as loud as it should
    // be, because a broken deadline hangs the server.
    rf::SandboxLimits limits;
    limits.wallSeconds = 0.75;
    limits.cpuSeconds  = 30;  // deliberately far above the wall deadline

    const auto  started = std::chrono::steady_clock::now();
    const auto  result  = rf::runSandboxed({"/bin/sh", "-c", "sleep 3600"}, ".", limits);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

    CHECK(result.outcome == rf::SandboxOutcome::TimedOut);
    CHECK(elapsed >= 0.75);
    // The kill is prompt, not eventual. A supervisor that took ten times the
    // deadline to act would still "work" and would still be useless.
    CHECK(elapsed < 3.0);
    CHECK(result.describe().find("SIGKILL") != std::string::npos);
}

TEST_CASE("the deadline is enforced against the whole process group") {
    // A child that forks and exits leaves its children running with nothing left
    // to kill them by, unless the supervisor signals the group. `sh` exits
    // immediately here; the grandchild is what has to die.
    rf::SandboxLimits limits;
    limits.wallSeconds = 0.75;

    const auto result = rf::runSandboxed({"/bin/sh", "-c", "sleep 3600 & exit 0"}, ".", limits);
    // The shell exits 0 at once, but its child holds the stdout pipe open, so
    // the supervisor cannot see EOF and rides the deadline out. Either outcome
    // is correct; what is not correct is returning while the grandchild lives.
    CHECK((result.outcome == rf::SandboxOutcome::TimedOut ||
           result.outcome == rf::SandboxOutcome::Ok));
    CHECK(result.wallSeconds < 3.0);
}

TEST_CASE("a child that dies on a signal is a result, not an error") {
    rf::SandboxLimits limits;
    limits.wallSeconds = 5.0;

    const auto result = rf::runSandboxed({"/bin/sh", "-c", "kill -SEGV $$"}, ".", limits);
    CHECK(result.outcome == rf::SandboxOutcome::Signalled);
    CHECK(result.signal == SIGSEGV);
    CHECK(result.describe().find("signal") != std::string::npos);
}

TEST_CASE("a nonzero exit is distinguished from a crash") {
    rf::SandboxLimits limits;
    limits.wallSeconds = 5.0;

    const auto result = rf::runSandboxed({"/bin/sh", "-c", "exit 3"}, ".", limits);
    CHECK(result.outcome == rf::SandboxOutcome::NonZeroExit);
    CHECK(result.exitCode == 3);
    CHECK_FALSE(result.ok());
}

TEST_CASE("output is captured, and a flood is cut off rather than buffered") {
    SUBCASE("ordinary output comes back on the right stream") {
        rf::SandboxLimits limits;
        limits.wallSeconds = 5.0;
        const auto result  = rf::runSandboxed({"/bin/sh", "-c", "echo out; echo err >&2"}, ".",
                                              limits);
        CHECK(result.ok());
        CHECK(result.out == "out\n");
        CHECK(result.err == "err\n");
        CHECK_FALSE(result.outputTruncated);
    }

    SUBCASE("an endless printer is killed instead of exhausting the parent") {
        // Pipes are not covered by RLIMIT_FSIZE, so without the output cap this
        // child would fill the *server's* memory rather than its own disk.
        rf::SandboxLimits limits;
        limits.wallSeconds   = 10.0;
        limits.maxOutputBytes = 64 * 1024;

        const auto result =
            rf::runSandboxed({"/bin/sh", "-c", "yes rocketfight"}, ".", limits);
        CHECK(result.outputTruncated);
        CHECK(result.out.size() <= 64 * 1024);
        CHECK(result.wallSeconds < 5.0);  // cut off long before the deadline
    }
}

TEST_CASE("a spawn failure is reported, never thrown") {
    rf::SandboxLimits limits;
    limits.wallSeconds = 5.0;

    SUBCASE("no such binary") {
        const auto result = rf::runSandboxed({"/nonexistent/rocketfight_runner"}, ".", limits);
        // execve fails inside the child, which exits 126 after saying why.
        CHECK_FALSE(result.ok());
        CHECK(result.err.find("execve") != std::string::npos);
    }

    SUBCASE("empty argv") {
        const auto result = rf::runSandboxed({}, ".", limits);
        CHECK(result.outcome == rf::SandboxOutcome::SpawnFailed);
        CHECK(result.diagnostic == "empty argv");
    }
}

// ---------------------------------------------------------------------------
// The two together: the real runner binary, loading a real broken submission,
// under the real supervisor. Everything above tests one half; this tests that
// the halves are actually wired to each other.
// ---------------------------------------------------------------------------

TEST_CASE("the real runner survives a submission that segfaults, and says so") {
    rf::SandboxLimits limits;
    limits.wallSeconds = 10.0;

    const std::vector<std::string> argv{rf::kTestRunnerPath, "--rocket=classic", "--mode=accel",
                                        "--seconds=1", "--library=" + fixture("segfault")};
    const auto result = rf::runSandboxed(argv, ".", limits,
                                         {"PATH=/usr/bin:/bin",
                                          std::string("ROCKETFIGHT_ROCKETS=") + rf::kTestRocketDir,
                                          "LC_ALL=C"});

    // The submission loads and initialises cleanly and then stores through a
    // null pointer on the first control step. There is nothing the host could
    // have validated to prevent this, which is the entire argument for putting
    // it in a process the host was already prepared to lose.
    CHECK(result.outcome == rf::SandboxOutcome::Signalled);
    CHECK(result.signal == SIGSEGV);
    CHECK(result.wallSeconds < 5.0);
}

TEST_CASE("the real runner hanging on a submission is killed at the deadline") {
    rf::SandboxLimits limits;
    limits.wallSeconds = 1.5;
    limits.cpuSeconds  = 60;  // the hang burns no CPU; only the wall clock ends it

    const std::vector<std::string> argv{rf::kTestRunnerPath, "--rocket=classic", "--mode=accel",
                                        "--seconds=60", "--library=" + fixture("sleep")};
    const auto result = rf::runSandboxed(argv, ".", limits,
                                         {"PATH=/usr/bin:/bin",
                                          std::string("ROCKETFIGHT_ROCKETS=") + rf::kTestRocketDir,
                                          "LC_ALL=C"});

    CHECK(result.outcome == rf::SandboxOutcome::TimedOut);
    CHECK(result.wallSeconds >= 1.5);
    CHECK(result.wallSeconds < 4.0);
    // Nothing usable came back, which is the correct amount: the runner prints
    // its JSON at the end of a run and this run never had an end.
    CHECK(result.out.empty());
}

TEST_CASE("a submission the runner refuses to load exits with JSON on stdout") {
    // The parent parses stdout and nothing else, so a rejection that went to
    // stderr would reach the operator's log and never reach the leaderboard.
    rf::SandboxLimits limits;
    limits.wallSeconds = 10.0;

    const std::vector<std::string> argv{rf::kTestRunnerPath, "--rocket=classic", "--mode=accel",
                                        "--seconds=1", "--library=" + fixture("wrong_version")};
    const auto result = rf::runSandboxed(argv, ".", limits,
                                         {"PATH=/usr/bin:/bin",
                                          std::string("ROCKETFIGHT_ROCKETS=") + rf::kTestRocketDir,
                                          "LC_ALL=C"});

    CHECK(result.outcome == rf::SandboxOutcome::NonZeroExit);
    CHECK(result.out.find("\"error\"") != std::string::npos);
    CHECK(result.out.find("ABI version mismatch") != std::string::npos);
}

TEST_CASE("the environment handed to a child is the one it was given, not the server's") {
    // A server's own environment routinely holds tokens and paths that have
    // nothing to do with flying a rocket. Replacing it rather than adding to it
    // is the whole reason this parameter exists, so it is worth an assertion.
    rf::SandboxLimits limits;
    limits.wallSeconds = 5.0;

    const auto result = rf::runSandboxed({"/bin/sh", "-c", "echo \"[$PATH][$HOME]\""}, ".", limits,
                                         {"PATH=/usr/bin:/bin", "HOME=/nowhere"});
    CHECK(result.ok());
    CHECK(result.out == "[/usr/bin:/bin][/nowhere]\n");
}
