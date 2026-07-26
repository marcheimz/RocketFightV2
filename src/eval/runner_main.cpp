#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "control/AbiFlyByWire.hpp"
#include "data/RocketCatalogue.hpp"
#include "eval/BenchmarkJson.hpp"
#include "eval/BenchmarkRunner.hpp"

// ---------------------------------------------------------------------------
// rocketfight_runner -- one benchmark job, one process, JSON on stdout.
//
// Why a separate binary at all: a submission's rf_resolve can spin forever,
// recurse until the stack is gone, or free a pointer it never allocated. None
// of those can be contained from inside the process they happen in -- there is
// no portable way to kill a hung function call, and a corrupted heap is
// corrupted for everybody sharing it. So the only honest containment is a
// process boundary, and this is the thing on the far side of it.
//
// It therefore does the least it possibly can: load one rocket, load one
// library, run one configuration, print one object, exit. Everything about
// supervision -- the deadline, the rlimits, the SIGKILL, the scratch directory
// -- belongs to the parent, in src/server/Sandbox.cpp, because a child cannot
// be trusted to enforce limits on itself.
//
// One job per process rather than the whole matrix, deliberately. It costs a
// fork per configuration (a few hundred microseconds against a 20 ms run) and
// buys per-job attribution: "crashed on norcs in velocity mode" instead of
// "crashed". It also makes the determinism check stronger than it would
// otherwise be, since the two runs being compared are two *different processes*
// -- so a submission that keys off an address or a pid is caught, which it
// would not be if both runs shared an address space.
// ---------------------------------------------------------------------------

namespace {

using Json = rf::BenchJson;

struct Options {
    std::string      library;   // path to the submission .so; empty = built-in
    std::string      rocket;
    rf::Intent::Mode mode{rf::Intent::Mode::Acceleration};
    std::uint64_t    seed{rf::BenchmarkIntentSource::kDefaultSeed};
    rf::Real         seconds{60.0};
    int              indent{-1};  // compact by default: a parser reads this
    bool             ok{true};
    std::string      error;
};

bool startsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

Options parseOptions(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        try {
            if (startsWith(arg, "--library=")) {
                o.library = arg.substr(10);
            } else if (startsWith(arg, "--rocket=")) {
                o.rocket = arg.substr(9);
            } else if (startsWith(arg, "--seed=")) {
                o.seed = std::stoull(arg.substr(7), nullptr, 0);
            } else if (startsWith(arg, "--seconds=")) {
                o.seconds = std::stod(arg.substr(10));
            } else if (startsWith(arg, "--mode=")) {
                if (!rf::intentModeFromString(arg.substr(7), o.mode)) {
                    o.ok    = false;
                    o.error = "unknown mode '" + arg.substr(7) + "', expected accel or velocity";
                }
            } else if (arg == "--pretty") {
                o.indent = 2;
            } else {
                o.ok    = false;
                o.error = "unknown option '" + arg + "'";
            }
        } catch (const std::exception&) {
            o.ok    = false;
            o.error = "malformed option '" + arg + "'";
        }
    }
    if (o.seconds <= rf::Real(0)) {
        o.ok    = false;
        o.error = "--seconds must be positive";
    }
    if (o.seconds > rf::kMaxBenchmarkSeconds) {
        o.ok    = false;
        o.error = "--seconds must be at most " +
                  std::to_string(static_cast<int>(rf::kMaxBenchmarkSeconds)) +
                  ": past roughly 450 s vehicles drift outside the world bound, which the "
                  "benchmark deliberately does not check";
    }
    if (o.rocket.empty()) {
        o.ok    = false;
        o.error = "--rocket=NAME is required";
    }
    return o;
}

// Errors are JSON on stdout too. The parent parses stdout and nothing else, so
// a message that went to stderr would reach the log and not the leaderboard.
int fail(const std::string& message) {
    std::cout << Json{{"error", message}}.dump() << "\n";
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
    const Options opts = parseOptions(argc, argv);
    if (!opts.ok) return fail(opts.error);

    const rf::RocketCatalogue& catalogue = rf::RocketCatalogue::instance();
    const rf::RocketSpec*      rocket    = catalogue.find(opts.rocket);
    if (rocket == nullptr) {
        return fail("unknown rocket '" + opts.rocket + "' (looked in " +
                    rf::RocketCatalogue::defaultRocketDir().string() + ")");
    }

    rf::BenchmarkConfig config;
    config.mode = opts.mode;

    rf::BenchmarkSpec spec;
    spec.rocket   = *rocket;
    spec.config   = config;
    spec.seed     = opts.seed;
    spec.maxTicks = static_cast<std::uint32_t>(opts.seconds * rf::kTickRate);

    // The whole body is guarded because a submission built as C++ may let an
    // exception escape rf_resolve even though the ABI forbids it. If it unwinds
    // this far, "the submission threw" is a result worth reporting; if it does
    // not unwind cleanly the process dies and the parent reports that instead.
    try {
        std::unique_ptr<rf::FlyByWire> fbw;
        if (!opts.library.empty()) {
            std::string error;
            auto        loaded = rf::AbiFlyByWire::load(opts.library, spec.rocket, error);
            if (!loaded) return fail(error);
            fbw = std::move(loaded);
        } else {
            // No --library is the baseline: the built-in RocketFlyByWire, run
            // through the identical loop, so "beat the reference" is a
            // comparison and not a claim.
            fbw = rf::makeFlyByWire(spec.rocket);
        }

        const rf::BenchmarkRun run = rf::runBenchmark(spec, std::move(fbw));

        Json out = rf::toJson(run, rf::ScoreWeights{});
        out["library"] = opts.library.empty() ? std::string("<builtin>") : opts.library;
        std::cout << out.dump(opts.indent) << "\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        return fail(std::string("submission threw: ") + e.what());
    } catch (...) {
        return fail("submission threw a non-std exception");
    }
}
