// ---------------------------------------------------------------------------
// The reference submission: the project's own RocketFlyByWire, wrapped in the C
// ABI. It exists for two reasons.
//
// First, it is a worked example that compiles: everything SUBMISSIONS.md
// describes is done here once, in the order the host calls it.
//
// Second, it is the baseline. The server scores it like anything else, so
// "better than the built-in fly-by-wire" is a number on a leaderboard rather
// than an assertion -- and a submission that cannot beat it has learned
// something useful about its own approach.
//
// Note what crosses the boundary: nothing. This file owns a C++ object and a
// C++ standard library, and none of that is visible to the host, which sees
// five C functions and four POD structs. That is the property that makes a
// Rust or a hand-written-C submission exactly as legitimate as this one.
// ---------------------------------------------------------------------------

#include "rocketfight/flybywire_abi.h"

#include <cmath>
#include <exception>
#include <memory>
#include <optional>

#include "control/RocketFlyByWire.hpp"

namespace {

// One instance per process, because the host runs one episode per process. A
// submission that wanted per-episode state on the heap would allocate it in
// rf_init and free it in rf_shutdown, exactly as this does.
std::unique_ptr<rf::FlyByWire> g_fbw;
rf::RocketSpec                 g_spec;

rf::Vec2 fromAbi(const RfVec2& v) { return rf::Vec2{v.x, v.y}; }

// The ABI's spec is not rf::RocketSpec: it carries no thruster names, and its
// inertia is a field rather than a method. Rebuilding the host's struct from it
// is the price of the boundary being narrow, and it is a small price.
rf::RocketSpec fromAbi(const RfRocketSpec& s) {
    rf::RocketSpec out;
    out.name   = rf::RocketName::from("submitted");
    out.mass   = s.mass;
    out.length = s.length;
    out.width  = s.width;

    const int n = s.thruster_count < RF_MAX_THRUSTERS ? s.thruster_count : RF_MAX_THRUSTERS;
    for (int i = 0; i < n; ++i) {
        const RfThruster& t = s.thrusters[i];
        rf::Thruster      d{};
        d.mountLocal     = fromAbi(t.mount);
        d.directionLocal = t.direction;
        d.minThrust      = t.min_thrust;
        d.maxThrust      = t.max_thrust;
        d.ignitionTime   = t.ignition_time;
        d.rampUpRate     = t.ramp_up;
        d.rampDownRate   = t.ramp_down;
        d.maxGimbal      = t.max_gimbal;
        d.gimbalRate     = t.gimbal_rate;
        out.add(d);
    }
    return out;
}

rf::Observation fromAbi(const RfObservation& o) {
    rf::Observation out;
    out.tick        = o.tick;
    out.time        = o.time;
    out.pos         = fromAbi(o.pos);
    out.vel         = fromAbi(o.vel);
    out.angle       = o.angle;
    out.angVel      = o.ang_vel;
    out.maxAccel    = o.max_accel;
    out.maxAngAccel = o.max_ang_accel;

    const int n = o.thruster_count < RF_MAX_THRUSTERS ? o.thruster_count : RF_MAX_THRUSTERS;
    out.thrusterCount = static_cast<std::size_t>(n < 0 ? 0 : n);
    for (int i = 0; i < n; ++i) {
        rf::ThrusterState& d = out.actuators[static_cast<std::size_t>(i)];
        d.thrust             = o.actuators[i].thrust;
        d.gimbalAngle        = o.actuators[i].gimbal_angle;
        d.ignitionTimer      = o.actuators[i].ignition_timer;
        d.lit                = o.actuators[i].lit != 0;
    }
    return out;
}

rf::Intent fromAbi(const RfIntent& i) {
    rf::Intent out;
    out.mode = i.mode == RF_MODE_VELOCITY ? rf::Intent::Mode::Velocity
                                          : rf::Intent::Mode::Acceleration;
    out.vector = fromAbi(i.vector);
    if (i.has_facing != 0) out.facing = i.facing;
    out.fire = i.fire != 0;
    return out;
}

}  // namespace

extern "C" {

RF_EXPORT int rf_abi_version(void) { return RF_ABI_VERSION; }

RF_EXPORT int rf_init(const RfRocketSpec* spec) {
    // The catch is not decoration. An exception escaping into the host's C
    // frame is undefined behaviour, and "the controller declined the vehicle"
    // is a result the server can print; a corrupted unwind is not.
    try {
        if (spec == nullptr || spec->thruster_count <= 0) return 1;
        g_spec = fromAbi(*spec);
        g_fbw  = rf::makeFlyByWire(g_spec);
        return g_fbw ? 0 : 2;
    } catch (const std::exception&) {
        return 3;
    } catch (...) {
        return 4;
    }
}

RF_EXPORT void rf_reset(void) {
    if (g_fbw) g_fbw->reset();
}

RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs, RfControlInput* out) {
    if (intent == nullptr || obs == nullptr || out == nullptr) return;
    // Written by the host as zeroed, so an early return here is a well-defined
    // "everything off" rather than whatever was in the caller's stack.
    if (!g_fbw) return;

    try {
        const rf::ControlInput c = g_fbw->resolve(fromAbi(*intent), fromAbi(*obs));
        for (std::size_t i = 0; i < rf::kMaxThrusters; ++i) {
            out->level[i]  = c.level[i];
            out->gimbal[i] = c.gimbal[i];
        }
        out->fire = c.fire ? 1 : 0;
    } catch (...) {
        // A thrown resolve leaves the demand at zero for this control period.
        // Coasting for 10 ms is survivable; unwinding into C is not.
    }
}

RF_EXPORT void rf_shutdown(void) { g_fbw.reset(); }

}  // extern "C"
