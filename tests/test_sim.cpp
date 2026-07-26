#include <doctest/doctest.h>

#include <chrono>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "core/WorldConfig.hpp"
#include "data/RocketCatalogue.hpp"
#include "sim/SimulationLoop.hpp"
#include "sync/Channels.hpp"

using namespace rf;

namespace {

// The loop is real-time paced, so asserting on the very next published snapshot
// would be asserting on the scheduler. Poll instead, with a limit that is
// generous by three orders of magnitude and still finishes instantly in practice.
template <typename Pred>
bool waitFor(StateChannel& state, Pred pred) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        if (state.acquire() && pred(state.read())) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

auto flying(std::string_view name) {
    return [name](const Snapshot& s) {
        return !s.rockets.empty() && s.rockets.front().spec.name.view() == name;
    };
}

}  // namespace

TEST_CASE("the pilot can change vehicle without the render side touching the world") {
    std::vector<WorldConfig> roster;
    for (const RocketSpec& spec : RocketCatalogue::instance().all()) {
        roster.push_back(defaultWorld(spec));
    }
    REQUIRE(roster.size() >= 2);

    const std::string first(roster.front().rockets.front().spec.name.view());
    const std::string second(roster[1].rockets.front().spec.name.view());
    const std::string last(roster.back().rockets.front().spec.name.view());

    StateChannel state;
    CommandQueue commands;

    SimulationLoop sim(roster.front(), state, commands);
    sim.setWorldRoster(roster, 0);

    std::thread thread([&sim] { sim.run(); });

    REQUIRE(waitFor(state, flying(first)));

    // The only channel the pilot has is the command queue: nothing here holds a
    // reference to the World, and the switch has to happen entirely on the far
    // side of the queue.
    commands.push(Command::makeButton(InputButton::NextVehicle, true));
    commands.push(Command::makeButton(InputButton::NextVehicle, false));
    CHECK(waitFor(state, flying(second)));

    // And it wraps, in both directions, rather than sticking at the ends.
    commands.push(Command::makeButton(InputButton::PrevVehicle, true));
    commands.push(Command::makeButton(InputButton::PrevVehicle, false));
    CHECK(waitFor(state, flying(first)));

    commands.push(Command::makeButton(InputButton::PrevVehicle, true));
    commands.push(Command::makeButton(InputButton::PrevVehicle, false));
    CHECK(waitFor(state, flying(last)));

    sim.stop();
    thread.join();

    CHECK(commands.dropped() == 0);
}

TEST_CASE("a one-vehicle roster is not a special case anyone has to remember") {
    std::vector<WorldConfig> roster{defaultWorld(RocketCatalogue::instance().all().front())};
    const std::string        only(roster.front().rockets.front().spec.name.view());

    StateChannel state;
    CommandQueue commands;

    SimulationLoop sim(roster.front(), state, commands);
    sim.setWorldRoster(roster, 0);

    std::thread thread([&sim] { sim.run(); });

    commands.push(Command::makeButton(InputButton::NextVehicle, true));
    commands.push(Command::makeButton(InputButton::NextVehicle, false));
    CHECK(waitFor(state, flying(only)));

    sim.stop();
    thread.join();
}
