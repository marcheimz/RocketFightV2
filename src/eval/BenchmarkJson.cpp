#include "eval/BenchmarkJson.hpp"

#include <iomanip>
#include <sstream>

namespace rf {

std::string hex64(std::uint64_t v) {
    std::ostringstream os;
    os << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
    return os.str();
}

BenchJson toJson(const BenchmarkRun& r, const ScoreWeights& weights) {
    const BenchmarkMetrics& m = r.metrics;
    return BenchJson{
        {"rocket", r.rocket},
        {"mode", intentModeName(r.mode)},
        {"seed", r.seed},
        {"ticks", r.ticks},
        {"seconds", m.seconds},
        {"state_hash", hex64(r.stateHash)},
        {"metrics",
         BenchJson{
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
        {"score", compositeScore(m, weights)},
    };
}

BenchJson toJson(const BenchmarkConfig& c, std::uint64_t seed, Real seconds) {
    return BenchJson{
        {"mode", intentModeName(c.mode)},
        {"seed", seed},
        {"seconds", seconds},
        {"hold_seconds", {c.holdMinSeconds, c.holdMaxSeconds}},
        {"accel_fraction_of_max", {c.accelMin, c.accelMax}},
        {"velocity_max_mps", c.speedMax},
        {"accel_settle_fraction", c.accelSettleFraction},
        {"velocity_settle_tolerance_mps", c.velocitySettleTolerance},
        {"control_hz", kTickRate / Real(kControlEvery)},
    };
}

BenchJson toJson(const ScoreWeights& w) {
    return BenchJson{
        {"tracking", w.tracking},
        {"settling", w.settling},
        {"impulse", w.impulse},
        {"wander", w.wander},
    };
}

}  // namespace rf
