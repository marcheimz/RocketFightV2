#pragma once

#include <cstdint>
#include <memory>

#include "core/ControlInput.hpp"
#include "core/Intent.hpp"
#include "core/Observation.hpp"
#include "core/WorldConfig.hpp"

namespace rf {

// ---------------------------------------------------------------------------
// Layer 1: where an Intent comes from. A human on a gamepad, or a policy that
// reasons about where to go without wanting to know about thrusters.
// ---------------------------------------------------------------------------
class IntentSource {
public:
    virtual ~IntentSource() = default;

    virtual Intent intent(const Observation& obs) = 0;
    virtual void   reset(std::uint64_t /*seed*/, const WorldConfig& /*env*/) {}
};

// ---------------------------------------------------------------------------
// Layer 2: Intent -> actuators. Specific to one vehicle, because this is where
// thruster count, gimbal range, RCS authority and inertia actually matter.
//
// Stateful: it closes a loop on attitude, so it has memory and needs a reset.
// ---------------------------------------------------------------------------
class FlyByWire {
public:
    virtual ~FlyByWire() = default;

    virtual ControlInput resolve(const Intent& intent, const Observation& obs) = 0;
    virtual void         reset() = 0;
};

// ---------------------------------------------------------------------------
// Layer 3: what the simulation actually talks to. Everything above composes
// down to this, and the simulation knows nothing else.
// ---------------------------------------------------------------------------
class Controller {
public:
    virtual ~Controller() = default;

    virtual ControlInput evaluate(const Observation& obs) = 0;
    virtual void         reset(std::uint64_t seed, const WorldConfig& env) = 0;

    // For HUDs and debugging: what the controller last wanted, if it thinks in
    // terms of Intent at all. An end-to-end policy legitimately has no answer.
    virtual const Intent* lastIntent() const { return nullptr; }
};

// Builds the fly-by-wire for a given vehicle. Defined in RocketFlyByWire.cpp.
std::unique_ptr<FlyByWire> makeFlyByWire(const RocketSpec& spec);

// Composes layers 1 and 2 into a Controller. A human pilot and an intent-level
// policy are the same object with a different IntentSource -- which is the
// property that lets them be compared on equal terms.
//
// By default the fly-by-wire is built at reset from whatever vehicle the world
// actually contains. That is what makes a policy portable: the same
// `seek-target` entry in the registry flies a lander or a gimbal-only rocket
// without knowing either exists -- including rockets added as JSON after this
// code was written.
class LayeredController final : public Controller {
public:
    explicit LayeredController(std::unique_ptr<IntentSource> source)
        : source_(std::move(source)), autoSelect_(true) {}

    LayeredController(std::unique_ptr<IntentSource> source, std::unique_ptr<FlyByWire> fbw)
        : source_(std::move(source)), fbw_(std::move(fbw)) {}

    ControlInput evaluate(const Observation& obs) override {
        if (!fbw_) return {};
        lastIntent_ = source_->intent(obs);
        return fbw_->resolve(lastIntent_, obs);
    }

    void reset(std::uint64_t seed, const WorldConfig& env) override {
        if (autoSelect_ && !env.rockets.empty()) {
            fbw_ = makeFlyByWire(env.rockets.front().spec);
        }
        source_->reset(seed, env);
        if (fbw_) fbw_->reset();
        lastIntent_ = Intent{};
    }

    const Intent* lastIntent() const override { return &lastIntent_; }

private:
    std::unique_ptr<IntentSource> source_;
    std::unique_ptr<FlyByWire>    fbw_;
    bool                          autoSelect_{false};
    Intent                        lastIntent_{};
};

}  // namespace rf
