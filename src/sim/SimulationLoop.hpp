#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "control/Benchmark.hpp"
#include "control/Controller.hpp"
#include "control/HumanInput.hpp"
#include "core/World.hpp"
#include "sync/Channels.hpp"

namespace rf {

// What the HUD shows about the benchmark. A plain value, copied out under
// relaxed atomics rather than travelling in the Snapshot: the Snapshot is the
// world's state and the benchmark is not part of the world, and a third
// lock-free channel would be real machinery for a text label.
//
// The fields are stored and read independently, so a reader can in principle see
// one of them a control step out of date. That is a display artefact nobody can
// perceive at 100 Hz, and it is the entire cost of not adding a channel.
struct BenchmarkHudState {
    bool          active{};
    bool          velocityMode{};
    Real          currentError{};
    Real          trackingErrorMean{};
    Real          settlingMean{};
    Real          impulse{};
    Real          attitudeWander{};
    Real          seconds{};
    std::uint32_t steps{};
    std::uint32_t stepsSettled{};
};

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

    // Safe to call from the render thread. See BenchmarkHudState.
    BenchmarkHudState benchmarkHud() const;

private:
    void handleCommand(const Command& c);
    void tickOnce();
    void publish(const SimStats& stats);
    void publishBenchmarkHud();

    // Arm or disarm the scripted sequence, and switch Intent mode for whoever is
    // flying. Simulation thread only.
    void setBenchmarkActive(bool on);
    void setVelocityMode(bool on);

    // Rebuild the world from the config it already holds, or from a neighbour in
    // the roster, and re-fit the controllers to it. Simulation thread only.
    void restart();
    void cycleVehicle(int step);

    Controller& activeController();

    World        world_;
    StateChannel* state_;
    CommandQueue* commands_;

    InputState                          input_;
    std::unique_ptr<LayeredController>  layered_;   // gamepad -> Intent -> fly-by-wire
    GamepadIntentSource*                human_{nullptr};   // owned by layered_
    std::unique_ptr<DirectHumanController> direct_; // gamepad -> actuators
    ControlInput                        held_{};    // zero-order held between control ticks

    // The same benchmark the headless binary runs. While it is armed it
    // *replaces* the pilot: it occupies the IntentSource slot the gamepad
    // normally does, which is exactly why the HUD has to say so.
    BenchmarkHarness bench_{};

    std::vector<WorldConfig> roster_;      // sim thread only, after run() starts
    std::size_t              current_{0};

    std::atomic<bool> running_{true};
    std::atomic<bool> flyByWire_{true};
    std::atomic<bool> benchmarkActive_{false};
    std::atomic<bool> velocityMode_{false};

    // std::atomic<Real> is lock-free on every platform this targets; if it ever
    // is not, the sim thread would take a mutex on the publish path, which is
    // the one thing this design does not do.
    static_assert(std::atomic<Real>::is_always_lock_free);

    std::atomic<Real>          hudError_{0};
    std::atomic<Real>          hudTracking_{0};
    std::atomic<Real>          hudSettling_{0};
    std::atomic<Real>          hudImpulse_{0};
    std::atomic<Real>          hudWander_{0};
    std::atomic<Real>          hudSeconds_{0};
    std::atomic<std::uint32_t> hudSteps_{0};
    std::atomic<std::uint32_t> hudSettled_{0};

    std::uint64_t resyncs_{0};
};

}  // namespace rf
