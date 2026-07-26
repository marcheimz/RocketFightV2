#include "control/AbiFlyByWire.hpp"

#include <dlfcn.h>

#include <cmath>
#include <cstring>

#include "rocketfight/flybywire_abi.h"

namespace rf {
namespace {

// dlsym hands back a void*, and converting that to a function pointer with a
// cast is exactly the conversion -Wpedantic exists to complain about. memcpy is
// the portable spelling and compiles to the same nothing.
template <typename Fn>
bool bindSymbol(void* handle, const char* name, Fn& out) {
    void* sym = dlsym(handle, name);
    if (sym == nullptr) return false;
    std::memcpy(&out, &sym, sizeof(sym));
    return true;
}

RfVec2 toAbi(const Vec2& v) { return RfVec2{v.x, v.y}; }

RfRocketSpec toAbi(const RocketSpec& spec) {
    RfRocketSpec out{};
    out.mass    = spec.mass;
    out.length  = spec.length;
    out.width   = spec.width;
    out.inertia = spec.inertia();

    // Truncation is impossible rather than tolerated: kMaxThrusters and
    // RF_MAX_THRUSTERS are the same number, and this is where anyone changing
    // one of them finds out.
    static_assert(kMaxThrusters == RF_MAX_THRUSTERS,
                  "the ABI's thruster capacity must match the simulation's");

    out.thruster_count = static_cast<int>(spec.count());
    for (std::size_t i = 0; i < spec.count(); ++i) {
        const Thruster& t = spec[i];
        RfThruster&     d = out.thrusters[i];
        d.mount           = toAbi(t.mountLocal);
        d.direction       = t.directionLocal;
        d.min_thrust      = t.minThrust;
        d.max_thrust      = t.maxThrust;
        d.ignition_time   = t.ignitionTime;
        d.ramp_up         = t.rampUpRate;
        d.ramp_down       = t.rampDownRate;
        d.max_gimbal      = t.maxGimbal;
        d.gimbal_rate     = t.gimbalRate;
    }
    // The thruster *name* is deliberately not in the ABI. Nothing in the
    // physics or the allocator reads it, and a submission that keyed off it
    // would stop being portable to a layout somebody else authored.
    return out;
}

RfObservation toAbi(const Observation& obs) {
    RfObservation out{};
    out.tick          = obs.tick;
    out.time          = obs.time;
    out.pos           = toAbi(obs.pos);
    out.vel           = toAbi(obs.vel);
    out.angle         = obs.angle;
    out.ang_vel       = obs.angVel;
    out.max_accel     = obs.maxAccel;
    out.max_ang_accel = obs.maxAngAccel;

    out.thruster_count = static_cast<int>(obs.thrusterCount);
    for (std::size_t i = 0; i < obs.thrusterCount && i < kMaxThrusters; ++i) {
        const ThrusterState& s = obs.actuators[i];
        RfThrusterState&     d = out.actuators[i];
        d.thrust               = s.thrust;
        d.gimbal_angle         = s.gimbalAngle;
        d.ignition_timer       = s.ignitionTimer;
        d.lit                  = s.lit ? 1 : 0;
    }
    return out;
}

RfIntent toAbi(const Intent& intent) {
    RfIntent out{};
    out.mode       = intent.mode == Intent::Mode::Velocity ? RF_MODE_VELOCITY : RF_MODE_ACCELERATION;
    out.vector     = toAbi(intent.vector);
    out.has_facing = intent.facing.has_value() ? 1 : 0;
    out.facing     = intent.facing.value_or(Real(0));
    out.fire       = intent.fire ? 1 : 0;
    return out;
}

// A submission's answer is untrusted arithmetic, not a promise. Non-finite
// values are replaced rather than clamped, because clamping a NaN keeps it.
Real sanitise(double v, Real lo, Real hi) {
    if (!std::isfinite(v)) return Real(0);
    return clamp(static_cast<Real>(v), lo, hi);
}

}  // namespace

std::unique_ptr<AbiFlyByWire> AbiFlyByWire::load(const std::string& path, const RocketSpec& spec,
                                                 std::string& error) {
    error.clear();

    // A bare filename would send dlopen hunting through the loader search path,
    // which turns "which library did we just evaluate?" into a question about
    // the host's environment. Submissions are named by path or not at all.
    if (path.find('/') == std::string::npos) {
        error = "library path must contain a '/': '" + path + "'";
        return nullptr;
    }

    // RTLD_LOCAL so the submission's symbols never join the global namespace:
    // two submissions in one process must not be able to see, or accidentally
    // interpose on, each other. RTLD_NOW so a missing dependency is a load
    // failure here rather than a crash at the first call.
    void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        const char* why = dlerror();
        error           = std::string("dlopen failed: ") + (why != nullptr ? why : "unknown error");
        return nullptr;
    }

    // Not make_unique: the constructor is private, and that is on purpose --
    // there is no valid AbiFlyByWire that did not come through this function.
    std::unique_ptr<AbiFlyByWire> self(new AbiFlyByWire());
    self->handle_ = handle;
    self->path_   = path;

    const struct {
        const char* name;
        bool        bound;
    } required[] = {
        {"rf_abi_version", bindSymbol(handle, "rf_abi_version", self->version_)},
        {"rf_init", bindSymbol(handle, "rf_init", self->init_)},
        {"rf_reset", bindSymbol(handle, "rf_reset", self->reset_)},
        {"rf_resolve", bindSymbol(handle, "rf_resolve", self->resolve_)},
        {"rf_shutdown", bindSymbol(handle, "rf_shutdown", self->shutdown_)},
    };
    for (const auto& sym : required) {
        if (!sym.bound) {
            error = std::string("missing required symbol '") + sym.name +
                    "' (C++ submissions must declare it extern \"C\")";
            return nullptr;
        }
    }

    // Version before anything else. A struct-layout mismatch does not announce
    // itself: it reads plausible garbage out of the middle of an observation
    // and looks like a controller that flies badly.
    const int version = self->version_();
    if (version != RF_ABI_VERSION) {
        error = "ABI version mismatch: library reports " + std::to_string(version) +
                ", host expects " + std::to_string(RF_ABI_VERSION);
        return nullptr;
    }

    const RfRocketSpec abiSpec = toAbi(spec);
    const int          rc      = self->init_(&abiSpec);
    if (rc != 0) {
        error = "rf_init declined the vehicle '" + std::string(spec.name.view()) +
                "' (returned " + std::to_string(rc) + ")";
        return nullptr;
    }

    self->thrusterCount_ = spec.count();
    self->reset_();
    return self;
}

AbiFlyByWire::~AbiFlyByWire() {
    // shutdown_ is only non-null once rf_init has succeeded, so a library that
    // failed to load is closed without being told to tear down state it never
    // built.
    if (shutdown_ != nullptr) shutdown_();
    if (handle_ != nullptr) dlclose(handle_);
}

ControlInput AbiFlyByWire::resolve(const Intent& intent, const Observation& obs) {
    const RfIntent      abiIntent = toAbi(intent);
    const RfObservation abiObs    = toAbi(obs);

    // Zeroed before the call, so a submission that writes nothing has commanded
    // "everything off" rather than inherited the last answer or read stack
    // garbage -- and a partially-filled output is still a defined one.
    RfControlInput abiOut{};
    resolve_(&abiIntent, &abiObs, &abiOut);

    ControlInput out{};
    for (std::size_t i = 0; i < thrusterCount_; ++i) {
        out.level[i]  = sanitise(abiOut.level[i], Real(0), Real(1));
        out.gimbal[i] = sanitise(abiOut.gimbal[i], Real(-1), Real(1));
    }
    // Slots past thrusterCount_ are left zero rather than copied. Nothing reads
    // them, and carrying a submission's scribbles forward would mean a HUD or a
    // future consumer could see a demand for a thruster the vehicle does not
    // have.
    out.fire = abiOut.fire != 0;
    return out;
}

void AbiFlyByWire::reset() {
    if (reset_ != nullptr) reset_();
}

}  // namespace rf
