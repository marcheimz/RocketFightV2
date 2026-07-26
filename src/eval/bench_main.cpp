#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "data/RocketCatalogue.hpp"
#include "eval/BenchmarkJson.hpp"
#include "eval/BenchmarkRunner.hpp"

// ---------------------------------------------------------------------------
// The headless benchmark: no window, no pacing, JSON on stdout.
//
// This is the binary a person points at the built-in fly-by-wire. The
// submission server drives rocketfight_runner instead, which runs one job per
// process under a wall-clock kill -- but both emit the same per-run object,
// from eval/BenchmarkJson.cpp, so a result archived from either is readable by
// the same tooling.
// ---------------------------------------------------------------------------

namespace {

using Json = rf::BenchJson;

struct Options {
    std::string   rocket;
    bool          all{false};
    std::uint64_t seed{rf::BenchmarkIntentSource::kDefaultSeed};
    rf::Real      seconds{60.0};
    rf::Intent::Mode mode{rf::Intent::Mode::Acceleration};
    int           indent{2};
    bool          ok{true};
    std::string   error;
};

bool startsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

Options parseOptions(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        try {
            if (startsWith(arg, "--rocket=")) {
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
            } else if (arg == "--all") {
                o.all = true;
            } else if (arg == "--compact") {
                o.indent = -1;
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
    return o;
}

int fail(const std::string& message) {
    std::cout << Json{{"error", message}}.dump(2) << "\n";
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
    const Options opts = parseOptions(argc, argv);
    if (!opts.ok) return fail(opts.error);

    const rf::RocketCatalogue& catalogue = rf::RocketCatalogue::instance();
    if (catalogue.empty()) {
        return fail("no rockets found in " + rf::RocketCatalogue::defaultRocketDir().string());
    }

    std::vector<rf::RocketSpec> fleet;
    if (opts.all) {
        fleet = catalogue.all();
    } else if (opts.rocket.empty()) {
        fleet.push_back(catalogue.all().front());
    } else if (const rf::RocketSpec* found = catalogue.find(opts.rocket)) {
        fleet.push_back(*found);
    } else {
        return fail("unknown rocket '" + opts.rocket + "'");
    }

    // Only the mode is a flag. Everything else is the shipped definition of the
    // task, and a submission server that could vary it per run would not be
    // ranking anybody against anybody.
    rf::BenchmarkConfig config;
    config.mode = opts.mode;

    const rf::ScoreWeights weights{};

    Json runs = Json::array();
    for (const rf::RocketSpec& rocket : fleet) {
        rf::BenchmarkSpec spec;
        spec.rocket   = rocket;
        spec.config   = config;
        spec.seed     = opts.seed;
        spec.maxTicks = static_cast<std::uint32_t>(opts.seconds * rf::kTickRate);

        runs.push_back(rf::toJson(rf::runBenchmark(spec), weights));
    }

    const Json out{
        {"benchmark", "flybywire-tracking"},
        {"version", 1},
        // Stated rather than assumed: in an orbit gravity would contribute to
        // every measured acceleration, and the score would be partly a score
        // for orbital mechanics.
        {"world", "zero-g"},
        {"config", rf::toJson(config, opts.seed, opts.seconds)},
        // Shipped with the results, because these are a guess. Anything ranking
        // an archive can re-weight the components without re-running a run.
        {"score_weights", rf::toJson(weights)},
        {"runs", runs},
    };

    std::cout << out.dump(opts.indent) << "\n";
    return EXIT_SUCCESS;
}
