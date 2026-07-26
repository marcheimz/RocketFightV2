#include "eval/BatchRunner.hpp"

#include <algorithm>
#include <atomic>
#include <thread>

#include "control/Registry.hpp"
#include "eval/EpisodeRunner.hpp"

namespace rf {

std::vector<EpisodeResult> runBatch(const std::vector<Job>& jobs, unsigned threads) {
    std::vector<EpisodeResult> results(jobs.size());
    if (jobs.empty()) return results;

    if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
    threads = std::min<unsigned>(threads, static_cast<unsigned>(jobs.size()));

    // A shared index rather than a static split: episodes end at wildly
    // different times once a controller can fly out of bounds early, and a
    // static split would leave most threads idle waiting for the slowest chunk.
    std::atomic<std::size_t> next{0};

    const auto worker = [&] {
        for (;;) {
            const std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= jobs.size()) return;

            const Job& job = jobs[i];
            auto controller = ControllerRegistry::instance().create(job.controller);
            if (!controller) {
                results[i].scenario   = job.scenario.name;
                results[i].controller = job.controller + " (unknown)";
                continue;
            }
            results[i] = runEpisode(job.scenario, *controller, job.controller);
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(threads);
    for (unsigned t = 0; t < threads; ++t) pool.emplace_back(worker);
    for (std::thread& t : pool) t.join();

    return results;
}

}  // namespace rf
