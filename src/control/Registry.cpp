#include "control/Registry.hpp"

#include <algorithm>

#include "control/RocketFlyByWire.hpp"
#include "control/ScriptedControllers.hpp"

namespace rf {

namespace {

// No fly-by-wire is named here. LayeredController picks the one matching
// whatever vehicle the scenario contains, so every policy below flies every
// rocket type without a single extra registry entry.
std::unique_ptr<Controller> makeLayered(std::unique_ptr<IntentSource> source) {
    return std::make_unique<LayeredController>(std::move(source));
}

}  // namespace

ControllerRegistry::ControllerRegistry() {
    add("idle", [] { return std::make_unique<IdleController>(); });

    add("spinburn", [] { return std::make_unique<SpinBurnController>(); });

    // Intent-level policies get the fly-by-wire for free -- they never mention
    // an actuator, and this is the line where that becomes concrete.
    add("seek-origin", [] { return makeLayered(std::make_unique<SeekPointIntentSource>()); });

    add("seek-far", [] {
        return makeLayered(std::make_unique<SeekPointIntentSource>(Vec2{Real(5000), Real(2000)}));
    });
}

ControllerRegistry& ControllerRegistry::instance() {
    static ControllerRegistry registry;
    return registry;
}

void ControllerRegistry::add(std::string name, Factory factory) {
    const auto it = std::find_if(entries_.begin(), entries_.end(),
                                 [&](const auto& e) { return e.first == name; });
    if (it != entries_.end()) {
        it->second = std::move(factory);
        return;
    }
    entries_.emplace_back(std::move(name), std::move(factory));
}

std::unique_ptr<Controller> ControllerRegistry::create(const std::string& name) const {
    const auto it = std::find_if(entries_.begin(), entries_.end(),
                                 [&](const auto& e) { return e.first == name; });
    return it == entries_.end() ? nullptr : it->second();
}

std::vector<std::string> ControllerRegistry::names() const {
    std::vector<std::string> out;
    out.reserve(entries_.size());
    for (const auto& e : entries_) out.push_back(e.first);
    return out;
}

}  // namespace rf
