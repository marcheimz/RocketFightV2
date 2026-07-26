#include "sim/SimulationLoop.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "control/RocketFlyByWire.hpp"

namespace rf {

namespace {

using Clock    = std::chrono::steady_clock;
using Nanos    = std::chrono::nanoseconds;
using TimePoint = Clock::time_point;

constexpr Nanos kTickPeriod{1'000'000};  // 1 ms

// If the loop is ever this far behind -- a debugger breakpoint, a laptop
// suspend, a scheduler stall -- stop trying to catch up and resynchronise.
// Simulating ten thousand ticks in a burst to "catch up" would freeze the
// process and then teleport everything, which is worse than admitting the time
// is gone.
constexpr int kMaxCatchUpTicks = 64;

// Sleeping is coarse: the kernel routinely overshoots a 1 ms request. Wake early
// and spin the last fraction, which costs a sliver of one core and buys an
// order of magnitude in timing accuracy.
constexpr Nanos kSpinMargin{150'000};  // 150 us

}  // namespace

SimulationLoop::SimulationLoop(WorldConfig cfg, StateChannel& state, CommandQueue& commands)
    : world_(std::move(cfg)), state_(&state), commands_(&commands) {
    // No vehicle named here either: the fly-by-wire is built from the world's
    // rocket at reset, so changing the vehicle changes how it flies without
    // touching the loop.
    layered_ = std::make_unique<LayeredController>(std::make_unique<GamepadIntentSource>(input_));
    direct_  = std::make_unique<DirectHumanController>(input_);

    restart();
}

void SimulationLoop::setWorldRoster(std::vector<WorldConfig> roster, std::size_t current) {
    roster_  = std::move(roster);
    current_ = roster_.empty() ? 0 : std::min(current, roster_.size() - 1);
}

Controller& SimulationLoop::activeController() {
    return flyByWire_.load(std::memory_order_relaxed) ? *layered_
                                                      : static_cast<Controller&>(*direct_);
}

void SimulationLoop::restart() {
    // The controllers are built against the vehicle: the fly-by-wire's deadband,
    // dead-time compensation and allocator all come out of the spec, so a reset
    // that skipped this would fly the new airframe with the old one's numbers.
    layered_->reset(world_.config().seed, world_.config());
    direct_->reset(world_.config().seed, world_.config());
    held_ = {};
}

void SimulationLoop::cycleVehicle(int step) {
    if (roster_.size() < 2) return;

    const auto n = static_cast<int>(roster_.size());
    current_     = static_cast<std::size_t>(((static_cast<int>(current_) + step) % n + n) % n);

    // A whole new World rather than a mutated one. Swapping the vehicle changes
    // the thruster count, the inertia and the actuator state all at once, and
    // "reset with a different spec" is exactly what constructing one is.
    world_ = World(roster_[current_]);
    restart();
}

void SimulationLoop::handleCommand(const Command& c) {
    // Edge-triggered actions first; everything else is level state.
    if (c.type == Command::Type::ButtonDown) {
        switch (c.button) {
            case InputButton::Reset:
                world_.reset();
                restart();
                return;
            case InputButton::NextVehicle:
                cycleVehicle(1);
                return;
            case InputButton::PrevVehicle:
                cycleVehicle(-1);
                return;
            case InputButton::ToggleDirect:
                flyByWire_.store(!flyByWire_.load(std::memory_order_relaxed),
                                 std::memory_order_relaxed);
                held_ = {};
                return;
            default:
                break;
        }
    }
    input_.apply(c);
}

void SimulationLoop::tickOnce() {
    // Controllers run at a decimation of the physics rate and their output is
    // held in between. A fixed tick count, so this is as deterministic as the
    // physics itself.
    if (world_.tick() % kControlEvery == 0) {
        held_ = activeController().evaluate(world_.observe(0));
    }
    world_.setControl(0, held_);
    world_.step(kTickDt);
}

void SimulationLoop::publish(const SimStats& stats) {
    Snapshot& slot = state_->writeSlot();
    world_.snapshot(slot);
    slot.stats = stats;
    state_->publish();
}

void SimulationLoop::run() {
    TimePoint nextTick = Clock::now();

    // Rate measurement over a sliding window, purely for the HUD.
    TimePoint     windowStart = Clock::now();
    Tick          windowTick0 = world_.tick();
    Nanos         windowBusy{0};
    SimStats      stats{};

    while (running_.load(std::memory_order_relaxed)) {
        Command c;
        while (commands_->pop(c)) handleCommand(c);

        const TimePoint beforeTicks = Clock::now();

        int stepped = 0;
        while (stepped < kMaxCatchUpTicks && Clock::now() >= nextTick) {
            tickOnce();
            nextTick += kTickPeriod;
            ++stepped;
        }

        const TimePoint afterTicks = Clock::now();
        if (stepped > 0) windowBusy += std::chrono::duration_cast<Nanos>(afterTicks - beforeTicks);

        // A resync means the loop *gave up*, which can only happen after a full
        // catch-up batch. Testing "are we behind" here instead would fire on
        // every healthy iteration: the loop exits the moment it is no longer
        // behind, and the clock has always advanced a little further by the time
        // it is read again.
        if (stepped >= kMaxCatchUpTicks && afterTicks >= nextTick) {
            nextTick = afterTicks;
            ++resyncs_;
        }

        if (stepped > 0) publish(stats);

        const auto windowAge = afterTicks - windowStart;
        if (windowAge >= std::chrono::milliseconds(250)) {
            const Real seconds = std::chrono::duration<Real>(windowAge).count();
            stats.tickRate   = static_cast<Real>(world_.tick() - windowTick0) / seconds;
            stats.loadFactor = std::chrono::duration<Real>(windowBusy).count() / seconds;
            stats.catchUpResyncs = resyncs_;

            windowStart = afterTicks;
            windowTick0 = world_.tick();
            windowBusy  = Nanos{0};
        }

        // Sleep most of the way to the deadline, then spin the remainder.
        const TimePoint wakeAt = nextTick - kSpinMargin;
        if (Clock::now() < wakeAt) {
            std::this_thread::sleep_until(wakeAt);
        } else {
            std::this_thread::yield();
        }
    }
}

}  // namespace rf
