#pragma once

#include <cstdint>

namespace rf {

// One typedef controls the precision of the entire simulation. See README:
// double is chosen because a long episode accumulates ~1e6 integration steps.
using Real     = double;
using Tick     = std::uint64_t;
using EntityId = std::uint32_t;

// The fixed physics step. This is a constant, never a frame time.
inline constexpr Real kTickDt   = Real(0.001);   // 1 ms -> 1000 Hz
inline constexpr Real kTickRate = Real(1000.0);

// Controllers run at a decimation of the physics rate and are zero-order held
// in between. Fixed tick count, so determinism is unaffected.
inline constexpr std::uint32_t kControlEvery = 10;                       // 100 Hz
inline constexpr Real          kControlDt    = kTickDt * kControlEvery;

// The world is bounded at 1000 km from the origin. Not a wall: crossing it
// terminates a headless episode so a runaway rocket cannot pace a batch run.
inline constexpr Real kWorldRadius = Real(1000.0e3);

inline constexpr Real kPi = Real(3.14159265358979323846);

}  // namespace rf
