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
    auto human = std::make_unique<GamepadIntentSource>(input_);
    human_     = human.get();
    layered_   = std::make_unique<LayeredController>(std::move(human));
    direct_    = std::make_unique<DirectHumanController>(input_);

    restart();
}

void SimulationLoop::setWorldRoster(std::vector<WorldConfig> roster, std::size_t current) {
    roster_  = std::move(roster);
    current_ = roster_.empty() ? 0 : std::min(current, roster_.size() - 1);
}

Controller& SimulationLoop::activeController() {
    // The benchmark outranks both human paths, including direct: what it is
    // measuring is the fly-by-wire, and letting the sticks half-fly it during a
    // scored run would produce a number that means nothing.
    if (benchmarkActive_.load(std::memory_order_relaxed)) return bench_.controller();

    return flyByWire_.load(std::memory_order_relaxed) ? static_cast<Controller&>(*layered_)
                                                      : static_cast<Controller&>(*direct_);
}

void SimulationLoop::restart() {
    // The controllers are built against the vehicle: the fly-by-wire's deadband,
    // dead-time compensation and allocator all come out of the spec, so a reset
    // that skipped this would fly the new airframe with the old one's numbers.
    layered_->reset(world_.config().seed, world_.config());
    direct_->reset(world_.config().seed, world_.config());

    // The benchmark refits too, and its metrics start over: a score that spanned
    // two airframes would not be a score for either.
    bench_.reset(world_.config());
    held_ = {};
}

void SimulationLoop::setBenchmarkActive(bool on) {
    benchmarkActive_.store(on, std::memory_order_relaxed);
    // Arming restarts the sequence from its seed, so the run in the window is
    // the same run the headless binary reports for this vehicle and seed.
    bench_.reset(world_.config());
    held_ = {};
    publishBenchmarkHud();
}

void SimulationLoop::setVelocityMode(bool on) {
    velocityMode_.store(on, std::memory_order_relaxed);

    // The mode belongs to the ship, not to whoever happens to be flying it: the
    // pilot's stick and the benchmark's script both mean the same thing by a
    // vector afterwards. KillVelocity is untouched and stays momentary.
    const Intent::Mode mode = on ? Intent::Mode::Velocity : Intent::Mode::Acceleration;
    human_->setMode(mode);
    bench_.setMode(mode);
    held_ = {};
    publishBenchmarkHud();
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
            case InputButton::ToggleBenchmark:
                setBenchmarkActive(!benchmarkActive_.load(std::memory_order_relaxed));
                return;
            case InputButton::ToggleVelocityMode:
                setVelocityMode(!velocityMode_.load(std::memory_order_relaxed));
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

    // Sampled after the step and at the full tick rate, exactly as the headless
    // runner does it, so the live numbers are the same numbers.
    if (benchmarkActive_.load(std::memory_order_relaxed)) {
        bench_.sample(world_.observe(0), kTickDt);
    }
}

void SimulationLoop::publish(const SimStats& stats) {
    Snapshot& slot = state_->writeSlot();
    world_.snapshot(slot);
    slot.stats = stats;
    state_->publish();

    publishBenchmarkHud();
}

void SimulationLoop::publishBenchmarkHud() {
    const BenchmarkMetrics& m = bench_.monitor().metrics();
    hudError_.store(bench_.monitor().currentError(), std::memory_order_relaxed);
    hudTracking_.store(m.trackingErrorMean, std::memory_order_relaxed);
    hudSettling_.store(m.settlingMean, std::memory_order_relaxed);
    hudImpulse_.store(m.impulse, std::memory_order_relaxed);
    hudWander_.store(m.attitudeWander, std::memory_order_relaxed);
    hudSeconds_.store(m.seconds, std::memory_order_relaxed);
    hudSteps_.store(m.steps, std::memory_order_relaxed);
    hudSettled_.store(m.stepsSettled, std::memory_order_relaxed);
}

BenchmarkHudState SimulationLoop::benchmarkHud() const {
    BenchmarkHudState s;
    s.active            = benchmarkActive_.load(std::memory_order_relaxed);
    s.velocityMode      = velocityMode_.load(std::memory_order_relaxed);
    s.currentError      = hudError_.load(std::memory_order_relaxed);
    s.trackingErrorMean = hudTracking_.load(std::memory_order_relaxed);
    s.settlingMean      = hudSettling_.load(std::memory_order_relaxed);
    s.impulse           = hudImpulse_.load(std::memory_order_relaxed);
    s.attitudeWander    = hudWander_.load(std::memory_order_relaxed);
    s.seconds           = hudSeconds_.load(std::memory_order_relaxed);
    s.steps             = hudSteps_.load(std::memory_order_relaxed);
    s.stepsSettled      = hudSettled_.load(std::memory_order_relaxed);
    return s;
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
