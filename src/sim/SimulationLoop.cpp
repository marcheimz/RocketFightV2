#include "sim/SimulationLoop.hpp"

#include <chrono>
#include <thread>

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
    layered_ = std::make_unique<LayeredController>(std::make_unique<GamepadIntentSource>(input_),
                                                   std::make_unique<RocketFlyByWire>());
    direct_  = std::make_unique<DirectHumanController>(input_);

    layered_->reset(world_.config().seed, world_.config());
    direct_->reset(world_.config().seed, world_.config());
}

Controller& SimulationLoop::activeController() {
    return flyByWire_.load(std::memory_order_relaxed) ? *layered_
                                                      : static_cast<Controller&>(*direct_);
}

void SimulationLoop::handleCommand(const Command& c) {
    // Edge-triggered actions first; everything else is level state.
    if (c.type == Command::Type::ButtonDown) {
        switch (c.button) {
            case InputButton::Reset:
                world_.reset();
                layered_->reset(world_.config().seed, world_.config());
                direct_->reset(world_.config().seed, world_.config());
                held_ = {};
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
