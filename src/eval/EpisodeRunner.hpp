#pragma once

#include <string>

#include "control/Controller.hpp"
#include "eval/Scenario.hpp"

namespace rf {

// Runs one episode headlessly, as fast as the CPU allows.
//
// The tick loop here is deliberately the same shape as the real-time one in
// SimulationLoop -- same fixed dt, same control decimation, same zero-order
// hold. If the two ever diverged, headless results would stop predicting what
// happens in the game, which would make the whole harness decorative.
EpisodeResult runEpisode(const Scenario& scenario, Controller& controller,
                         const std::string& controllerName = {});

}  // namespace rf
