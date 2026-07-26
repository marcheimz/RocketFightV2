#pragma once

#include <string>
#include <vector>

#include "eval/Scenario.hpp"

namespace rf {

// One unit of work: a scenario plus the name of the controller to fly it.
// The controller is named rather than passed, because each job constructs its
// own instance -- sharing one across threads would share its internal state.
struct Job {
    Scenario    scenario;
    std::string controller;
};

// Runs every job across a thread pool. Each job gets its own World and its own
// Controller; nothing is shared, so there is not a single lock in the physics.
//
// This is the substrate the whole point of the project rests on: N agents each
// write a controller, all of them run at once, and the results are comparable
// because every episode is bit-reproducible.
//
// threads == 0 means "use the hardware concurrency".
std::vector<EpisodeResult> runBatch(const std::vector<Job>& jobs, unsigned threads = 0);

}  // namespace rf
