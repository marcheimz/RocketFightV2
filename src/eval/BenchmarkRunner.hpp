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

// The longest run the benchmark is known to be meaningful over.
//
// The loop deliberately ignores World::anyOutOfBounds() so that every submission
// integrates over an identical window -- see the comment in BenchmarkRunner.cpp.
// The price is that nothing stops a vehicle leaving the 1000 km world bound, and
// measured across every rocket and 40 seeds, they begin doing so from around
// 450 s (earliest observed: interceptor at 449 s). At the 60 s default the
// margin is roughly 17x, with the worst case reaching 6% of the bound.
//
// So this is not a physics limit, it is the edge of the validated envelope, and
// callers that let a user pick the run length should refuse to go past it rather
// than silently produce a run whose vehicles are outside a boundary the rest of
// the engine treats as real.
inline constexpr Real kMaxBenchmarkSeconds = Real(300);

// Runs the vehicle's own fly-by-wire against the scripted sequence and reports
// what it cost. Deterministic in (rocket, config, seed): nothing here reads a
// clock, a global, or a thread id.
BenchmarkRun runBenchmark(const BenchmarkSpec& spec);

// The same run against a supplied fly-by-wire -- a submission loaded across the
// C ABI, say. The one above delegates to this with makeFlyByWire(spec.rocket),
// so the built-in baseline and a submission go through one loop rather than two
// that have to be kept in agreement.
BenchmarkRun runBenchmark(const BenchmarkSpec& spec, std::unique_ptr<FlyByWire> fbw);

}  // namespace rf
