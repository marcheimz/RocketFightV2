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

// Composes layers 1 and 2 into a Controller. A human pilot and an intent-level
// policy are the same object with a different IntentSource -- which is the
// property that lets them be compared on equal terms.
class LayeredController final : public Controller {
public:
    LayeredController(std::unique_ptr<IntentSource> source, std::unique_ptr<FlyByWire> fbw)
        : source_(std::move(source)), fbw_(std::move(fbw)) {}

    ControlInput evaluate(const Observation& obs) override {
        lastIntent_ = source_->intent(obs);
        return fbw_->resolve(lastIntent_, obs);
    }

    void reset(std::uint64_t seed, const WorldConfig& env) override {
        source_->reset(seed, env);
        fbw_->reset();
        lastIntent_ = Intent{};
    }

    const Intent* lastIntent() const override { return &lastIntent_; }

private:
    std::unique_ptr<IntentSource> source_;
    std::unique_ptr<FlyByWire>    fbw_;
    Intent                        lastIntent_{};
};

}  // namespace rf
