#pragma once

#include <cstdint>
#include <string>

#include "control/Benchmark.hpp"

namespace rf {

// One headless benchmark run: a vehicle, a task, a seed.
//
// The runner lives in eval/ and the sequence lives in control/ on purpose --
// see the comment at the top of control/Benchmark.hpp. This half is the half
// that builds a World and loops it, which is exactly what rf_sim already has its
// own version of and must not gain a second copy of.
struct BenchmarkSpec {
    RocketSpec      rocket{};
    BenchmarkConfig config{};
    std::uint64_t   seed{BenchmarkIntentSource::kDefaultSeed};
    std::uint32_t   maxTicks{60'000};  // 60 s of simulated time
};

struct BenchmarkRun {
    std::string   rocket;
    Intent::Mode  mode{Intent::Mode::Acceleration};
    std::uint64_t seed{};
    std::uint32_t ticks{};

    // The same fingerprint EpisodeResult carries, for the same reason: a run
    // that cannot be proved reproducible is a run nobody can be ranked on.
    std::uint64_t stateHash{};

    BenchmarkMetrics metrics{};
};

// Runs the vehicle's own fly-by-wire against the scripted sequence and reports
// what it cost. Deterministic in (rocket, config, seed): nothing here reads a
// clock, a global, or a thread id.
BenchmarkRun runBenchmark(const BenchmarkSpec& spec);

}  // namespace rf
