#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "control/Controller.hpp"
#include "control/HumanInput.hpp"
#include "core/World.hpp"
#include "sync/Channels.hpp"

namespace rf {

// Drives a World at a real-time-paced 1000 Hz on its own thread, publishing a
// snapshot after every batch of ticks and draining input before them.
//
// This class owns the only clock in the project. The World deliberately has no
// idea that wall time exists.
class SimulationLoop {
public:
    SimulationLoop(WorldConfig cfg, StateChannel& state, CommandQueue& commands);

    // The set of worlds NextVehicle/PrevVehicle cycle through, and which of them
    // is currently loaded. Must be called before run(); the roster is then read
    // only by the simulation thread, which is why swapping vehicles needs no
    // lock and no third shared object.
    //
    // The app builds these, because it owns the rocket catalogue and the sim
    // deliberately has no idea that rockets are loaded from files. All this
    // layer sees is a list of plain WorldConfig values.
    void setWorldRoster(std::vector<WorldConfig> roster, std::size_t current);

    // Blocks until stop(). Intended to be called on a dedicated thread.
    void run();

    // Safe to call from any thread.
    void stop() { running_.store(false, std::memory_order_relaxed); }

    bool flyByWireEnabled() const { return flyByWire_.load(std::memory_order_relaxed); }

private:
    void handleCommand(const Command& c);
    void tickOnce();
    void publish(const SimStats& stats);

    // Rebuild the world from the config it already holds, or from a neighbour in
    // the roster, and re-fit the controllers to it. Simulation thread only.
    void restart();
    void cycleVehicle(int step);

    Controller& activeController();

    World        world_;
    StateChannel* state_;
    CommandQueue* commands_;

    InputState                          input_;
    std::unique_ptr<Controller>         layered_;   // gamepad -> Intent -> fly-by-wire
    std::unique_ptr<DirectHumanController> direct_; // gamepad -> actuators
    ControlInput                        held_{};    // zero-order held between control ticks

    std::vector<WorldConfig> roster_;      // sim thread only, after run() starts
    std::size_t              current_{0};

    std::atomic<bool> running_{true};
    std::atomic<bool> flyByWire_{true};

    std::uint64_t resyncs_{0};
};

}  // namespace rf
