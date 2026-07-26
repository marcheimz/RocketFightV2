#pragma once

#include <memory>

#include "control/Controller.hpp"
#include "control/ThrustAllocator.hpp"

namespace rf {

// Layer 2, constructed from a rocket's spec at runtime.
//
// It hardcodes no layout. Whether the nose must lead, how much angular authority
// exists, whether that authority survives the engine being shut down, how long
// the engines take to light, and how tight an attitude deadband the thrust
// floors permit are all *read from the spec*. That is why one implementation
// flies a lander, a gimbal-only rocket and a twin-engined interceptor, and why
// they still feel like different vehicles.
//
// The actuators are the hard part. Thrust does not appear when commanded; it
// appears after an ignition delay and then ramps. It cannot be throttled below
// a floor, so fine corrections are not available at any price. Nozzles slew
// rather than snap. A controller that ignores all that keeps commanding
// corrections already on their way and oscillates.
class RocketFlyByWire final : public FlyByWire {
public:
    struct Gains {
        // Attitude follows a deceleration profile rather than a plain PD term:
        // a PD controller with saturating actuators overshoots badly on large
        // heading changes, while a profile that only ever asks for the fastest
        // rate the thrusters can still stop from does not.
        Real maxSlewRate{2.0};   // rad/s
        Real slewSafety{0.6};    // margin on the braking profile
        Real rateTau{0.35};      // s, first-order lag tracking the rate target

        Real velGain{2.0};       // 1/s, proportional law in velocity mode

        // Do not fire the main engine while pointing more than this far from the
        // requested direction. cos(60 degrees).
        Real thrustGateCos{0.5};

        // A vehicle with no attitude thrusters can only steer by deflecting a
        // lit engine, so it has to burn a little in order to turn at all.
        Real steeringThrottle{0.25};

        // ...and it may only count on part of that authority when planning how
        // hard it can brake, since throttle can drop at any moment.
        Real gimbalReliance{0.35};

        // How much of the ignition delay to anticipate. Commanding a correction
        // that will not begin for a quarter of a second and then judging it as
        // if it had is how a controller talks itself into a limit cycle.
        Real leadFraction{1.0};

        // Attitude deadband, as a multiple of the smallest correction the thrust
        // floors allow. Below this the controller stops firing entirely and lets
        // the vehicle drift, because the only alternative is a correction bigger
        // than the error.
        Real deadbandFactor{1.5};
        Real minDeadband{0.006};  // rad, floor for vehicles with no minimum thrust
    };

    explicit RocketFlyByWire(const RocketSpec& spec);
    RocketFlyByWire(const RocketSpec& spec, Gains gains);

    ControlInput resolve(const Intent& intent, const Observation& obs) override;

    // Nothing accumulates: the rate profile reads angular velocity from the
    // observation rather than integrating an error term.
    void reset() override {}

    const RocketSpec& spec() const { return spec_; }
    Real              attitudeDeadband() const { return deadband_; }

private:
    RocketSpec      spec_;
    ThrustAllocator allocator_;
    Gains           gains_{};

    // Derived from the layout once, rather than recomputed per tick.
    bool bodyMustPoint_{};
    bool gimbalOnly_{};
    Real brakeAuthority_{};
    Real deadband_{};
    Real ignitionLead_{};
    Real attitudeLag_{};   // dead time before a commanded correction exists
};

// Kept as a free function so callers do not need the concrete type. There is no
// longer a rocket-type switch behind it: a spec is all it takes.
std::unique_ptr<FlyByWire> makeFlyByWire(const RocketSpec& spec);

}  // namespace rf
