#include "core/World.hpp"

#include <cmath>

#include "core/Hash.hpp"

namespace rf {

World::World(WorldConfig cfg) : cfg_(std::move(cfg)), rng_(cfg_.seed) {
    reset();
}

void World::reset() {
    rockets_.clear();
    rockets_.reserve(cfg_.rockets.size());

    for (const RocketInit& init : cfg_.rockets) {
        rockets_.emplace_back(init.spec, init.pos, init.vel, init.angle, init.angVel);
    }

    tick_ = 0;
    rng_  = Rng(cfg_.seed);
}

void World::accumulateGravity(Rocket& r) const {
    if (cfg_.attractors.empty()) return;  // zero-g: the default world

    const Real eps2 = cfg_.softening * cfg_.softening;

    Vec2 accel{};
    for (const Attractor& a : cfg_.attractors) {
        const Vec2 d  = a.pos - r.body().pos;
        const Real r2 = lengthSq(d) + eps2;
        // Plummer-softened inverse square: a = mu * d / (|d|^2 + eps^2)^(3/2).
        const Real inv = a.mu / (r2 * std::sqrt(r2));
        accel += d * inv;
    }

    // Gravity is an acceleration; the body wants a force.
    r.body().applyForce(accel * r.spec().mass);
}

void World::step(Real dt) {
    // The world knows about gravity and time. It does not know how a rocket
    // works -- each vehicle applies its own controls, so adding a thruster
    // layout never means touching this loop.
    for (Rocket& r : rockets_) {
        r.body().clearForces();
        accumulateGravity(r);
        // Actuators advance first: thrust lags its demand, so what pushes the
        // hull this tick is the result of commands issued some ticks ago.
        r.stepActuators(dt);
        r.applyControls();
        r.body().integrate(dt);
    }
    ++tick_;
}

void World::setControl(EntityId id, const ControlInput& in) {
    if (id < rockets_.size()) rockets_[id].setControls(in);
}

Observation World::observe(EntityId id) const {
    Observation o;
    o.tick = tick_;
    o.time = time();

    if (id >= rockets_.size()) return o;

    const Rocket& r = rockets_[id];
    o.pos    = r.body().pos;
    o.vel    = r.body().vel;
    o.angle  = r.body().angle;
    o.angVel = r.body().angVel;

    o.maxAccel    = r.spec().maxForwardAccel();
    o.maxAngAccel = r.spec().maxAngAccel();

    o.thrusterCount = r.spec().count();
    o.actuators     = r.actuators();
    return o;
}

void World::snapshot(Snapshot& out) const {
    out.tick         = tick_;
    out.time         = time();
    out.boundsRadius = cfg_.boundsRadius;

    out.rockets.clear();
    out.rockets.reserve(rockets_.size());
    for (const Rocket& r : rockets_) {
        RocketView v;
        v.pos    = r.body().pos;
        v.vel    = r.body().vel;
        v.angle  = r.body().angle;
        v.angVel = r.body().angVel;
        v.spec      = r.spec();
        v.ctrl      = r.controls();
        v.actuators = r.actuators();
        out.rockets.push_back(v);
    }

    out.attractors = cfg_.attractors;
    // stats and command stay zeroed: the world has no clock, and no controller.
}

std::uint64_t World::hash() const {
    Hasher h;
    h.feed(tick_);
    for (const Rocket& r : rockets_) {
        h.feed(r.body().pos.x);
        h.feed(r.body().pos.y);
        h.feed(r.body().vel.x);
        h.feed(r.body().vel.y);
        h.feed(r.body().angle);
        h.feed(r.body().angVel);
        // Actuator state is simulation state: two runs that agree on position
        // but disagree on which engines are mid-ignition have diverged.
        for (std::size_t i = 0; i < r.spec().count(); ++i) {
            h.feed(r.actuators()[i].thrust);
            h.feed(r.actuators()[i].gimbalAngle);
            h.feed(r.actuators()[i].ignitionTimer);
        }
    }
    return h.value();
}

bool World::anyOutOfBounds() const {
    const Real r2 = cfg_.boundsRadius * cfg_.boundsRadius;
    for (const Rocket& r : rockets_) {
        if (lengthSq(r.body().pos) > r2) return true;
    }
    return false;
}

WorldConfig defaultWorld(const RocketSpec& rocket) {
    WorldConfig cfg;
    RocketInit init;
    init.spec  = rocket;
    init.pos   = {Real(0), Real(0)};
    init.vel   = {Real(0), Real(0)};
    init.angle = kPi / Real(2);  // nose up
    cfg.rockets.push_back(init);
    return cfg;
}

WorldConfig orbitWorld(const RocketSpec& rocket) {
    WorldConfig cfg;

    // A small dense body: 400 m radius, ~10 m/s^2 at the surface (mu = g*R^2).
    // Deliberately toy-scaled so an orbit is a thing you can watch rather than
    // something that takes 90 minutes.
    Attractor planet;
    planet.radius = Real(400.0);
    planet.mu     = Real(10.0) * planet.radius * planet.radius;  // 1.6e6
    planet.pos    = {Real(0), Real(0)};
    cfg.attractors.push_back(planet);

    // Circular orbit: v = sqrt(mu / r), perpendicular to the radius. At 2 km the
    // orbital speed is ~28 m/s and the period ~7.5 minutes, and local gravity is
    // 0.4 m/s^2 -- far below the rocket's 40 m/s^2, so a controller has real
    // authority to change the orbit rather than merely falling.
    const Real r = Real(2000.0);
    const Real v = std::sqrt(planet.mu / r);

    RocketInit init;
    init.spec  = rocket;
    init.pos   = {r, Real(0)};
    init.vel   = {Real(0), v};
    init.angle = kPi / Real(2);
    cfg.rockets.push_back(init);

    return cfg;
}

}  // namespace rf
