#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "data/RocketCatalogue.hpp"
#include "eval/BenchmarkRunner.hpp"

// ---------------------------------------------------------------------------
// The headless benchmark: no window, no pacing, JSON on stdout.
//
// This is the binary a submission server drives. Everything it prints is
// machine-readable, every run carries the seed and the state hash that make it
// reproducible, and the metrics are published as components with the score
// weights beside them -- so a ranking can be recomputed from an archive of
// results without re-running anything.
// ---------------------------------------------------------------------------

namespace {

// ordered_json, not json: the default keeps keys in a std::map, which sorts them
// and puts "runs" in the middle of the header. Parsers do not care, but a human
// checking a submission by eye does, and it costs nothing.
using Json = nlohmann::ordered_json;

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
    return o;
}

// uint64 as a hex string, not a JSON number: JSON's number type is a double
// almost everywhere it is parsed, and a 64-bit hash does not survive that.
std::string hex64(std::uint64_t v) {
    std::ostringstream os;
    os << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
    return os.str();
}

Json toJson(const rf::BenchmarkRun& r, const rf::ScoreWeights& weights) {
    const rf::BenchmarkMetrics& m = r.metrics;
    return Json{
        {"rocket", r.rocket},
        {"mode", rf::intentModeName(r.mode)},
        {"seed", r.seed},
        {"ticks", r.ticks},
        {"seconds", m.seconds},
        {"state_hash", hex64(r.stateHash)},
        {"metrics",
         Json{
             {"tracking_error_integral", m.trackingErrorIntegral},
             {"tracking_error_mean", m.trackingErrorMean},
             {"settling_time_mean_s", m.settlingMean},
             {"settling_time_worst_s", m.settlingWorst},
             {"settled_fraction", m.settledFraction()},
             {"steps", m.steps},
             {"steps_settled", m.stepsSettled},
             {"impulse_ns", m.impulse},
             {"mean_thrust_n", m.meanThrust()},
             {"attitude_wander_rad", m.attitudeWander},
             {"mean_ang_rate_rad_s", m.meanAngRate()},
         }},
        {"score", rf::compositeScore(m, weights)},
    };
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

        runs.push_back(toJson(rf::runBenchmark(spec), weights));
    }

    const Json out{
        {"benchmark", "flybywire-tracking"},
        {"version", 1},
        // Stated rather than assumed: in an orbit gravity would contribute to
        // every measured acceleration, and the score would be partly a score
        // for orbital mechanics.
        {"world", "zero-g"},
        {"config",
         Json{
             {"mode", rf::intentModeName(config.mode)},
             {"seed", opts.seed},
             {"seconds", opts.seconds},
             {"hold_seconds", {config.holdMinSeconds, config.holdMaxSeconds}},
             {"accel_fraction_of_max", {config.accelMin, config.accelMax}},
             {"velocity_max_mps", config.speedMax},
             {"accel_settle_fraction", config.accelSettleFraction},
             {"velocity_settle_tolerance_mps", config.velocitySettleTolerance},
             {"control_hz", rf::kTickRate / rf::Real(rf::kControlEvery)},
         }},
        // Shipped with the results, because these are a guess. Anything ranking
        // an archive can re-weight the components without re-running a run.
        {"score_weights",
         Json{
             {"tracking", weights.tracking},
             {"settling", weights.settling},
             {"impulse", weights.impulse},
             {"wander", weights.wander},
         }},
        {"runs", runs},
    };

    std::cout << out.dump(opts.indent) << "\n";
    return EXIT_SUCCESS;
}
