#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "control/Registry.hpp"
#include "core/World.hpp"
#include "data/RocketCatalogue.hpp"
#include "eval/BatchRunner.hpp"

namespace {

// Every registered controller flying every rocket type in both worlds, repeated
// so the batch is big enough to actually load the machine.
//
// The cross product is the point: not one entry in the registry names a vehicle,
// so this is a genuine sweep of policies against airframes rather than a list of
// hand-paired combinations.
std::vector<rf::Job> buildJobs(unsigned repeats) {
    std::vector<rf::Job> jobs;

    const rf::RocketCatalogue& catalogue = rf::RocketCatalogue::instance();

    for (unsigned r = 0; r < repeats; ++r) {
        for (const rf::RocketSpec& rocket : catalogue.all()) {
            const std::string vehicle(rocket.name.view());

            rf::Scenario empty;
            empty.name     = "zero-g/" + vehicle;
            empty.world    = rf::defaultWorld(rocket);
            empty.maxTicks = 60'000;  // 60 s

            rf::Scenario orbit;
            orbit.name     = "orbit/" + vehicle;
            orbit.world    = rf::orbitWorld(rocket);
            orbit.maxTicks = 60'000;

            for (const std::string& controller : rf::ControllerRegistry::instance().names()) {
                for (rf::Scenario s : {empty, orbit}) {
                    // Distinct seeds per repeat, so this is a sweep rather than
                    // the same episode run many times.
                    s.seed = 1000u + r;
                    jobs.push_back(rf::Job{s, controller});
                }
            }
        }
    }
    return jobs;
}

}  // namespace

int main(int argc, char** argv) {
    unsigned repeats = 8;
    unsigned threads = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.rfind("--repeats=", 0) == 0) repeats = static_cast<unsigned>(std::stoul(arg.substr(10)));
        if (arg.rfind("--threads=", 0) == 0) threads = static_cast<unsigned>(std::stoul(arg.substr(10)));
    }

    for (const rf::RocketLoadError& e : rf::RocketCatalogue::instance().errors()) {
        std::cerr << "rocket load error: " << e.file << ": " << e.message << "\n";
    }
    if (rf::RocketCatalogue::instance().empty()) {
        std::cerr << "no rockets found in " << rf::RocketCatalogue::defaultRocketDir() << "\n";
        return EXIT_FAILURE;
    }

    const std::vector<rf::Job> jobs = buildJobs(repeats);

    const auto start   = std::chrono::steady_clock::now();
    const auto results = rf::runBatch(jobs, threads);
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    std::uint64_t totalTicks = 0;
    for (const rf::EpisodeResult& r : results) totalTicks += r.ticks;

    std::cout << std::left << std::setw(20) << "scenario" << std::setw(14) << "controller"
              << std::setw(10) << "ticks" << std::setw(16) << "reason" << std::setw(20) << "hash"
              << "final position\n";
    std::cout << std::string(98, '-') << "\n";

    // One line per distinct (scenario, controller); the repeats exist to load
    // the pool, not to be read.
    for (std::size_t i = 0; i < results.size() && i < 16; ++i) {
        const rf::EpisodeResult& r = results[i];
        std::cout << std::left << std::setw(20) << r.scenario << std::setw(14) << r.controller
                  << std::setw(10) << r.ticks << std::setw(16) << rf::toString(r.reason)
                  << std::setw(20) << std::hex << r.stateHash << std::dec;
        if (!r.finalState.rockets.empty()) {
            const rf::Vec2 p = r.finalState.rockets.front().pos;
            std::cout << std::fixed << std::setprecision(1) << "(" << p.x << ", " << p.y << ")";
        }
        std::cout << "\n";
    }
    if (results.size() > 16) std::cout << "... " << (results.size() - 16) << " more\n";

    // Determinism check, for free: identical jobs must produce identical hashes.
    bool deterministic = true;
    for (std::size_t i = 0; i < results.size(); ++i) {
        for (std::size_t j = i + 1; j < results.size(); ++j) {
            const bool sameJob = jobs[i].controller == jobs[j].controller &&
                                 jobs[i].scenario.name == jobs[j].scenario.name &&
                                 jobs[i].scenario.seed == jobs[j].scenario.seed;
            if (sameJob && results[i].stateHash != results[j].stateHash) deterministic = false;
        }
    }

    std::cout << "\n" << jobs.size() << " episodes, " << totalTicks << " ticks in " << std::fixed
              << std::setprecision(2) << elapsed << " s  ->  "
              << std::setprecision(1) << (static_cast<double>(totalTicks) / elapsed / 1e6)
              << " M ticks/s   (" << (static_cast<double>(totalTicks) / elapsed / 1000.0)
              << "x real time)\n";
    std::cout << "determinism: " << (deterministic ? "identical jobs agree" : "MISMATCH") << "\n";

    return deterministic ? EXIT_SUCCESS : EXIT_FAILURE;
}
