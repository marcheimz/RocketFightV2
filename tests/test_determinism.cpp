#include <doctest/doctest.h>

#include <thread>
#include <vector>

#include "control/Registry.hpp"
#include "TestRockets.hpp"
#include "core/World.hpp"
#include "eval/BatchRunner.hpp"
#include "eval/EpisodeRunner.hpp"

using namespace rf;
using rf::testing::classic;

namespace {

Scenario orbitScenario(std::uint32_t ticks = 20'000) {
    Scenario s;
    s.name     = "orbit";
    s.world    = orbitWorld(classic());
    s.seed     = 4242;
    s.maxTicks = ticks;
    return s;
}

}  // namespace

TEST_CASE("the same episode twice produces the same bits") {
    auto a = ControllerRegistry::instance().create("seek-far");
    auto b = ControllerRegistry::instance().create("seek-far");
    REQUIRE(a);
    REQUIRE(b);

    const EpisodeResult ra = runEpisode(orbitScenario(), *a);
    const EpisodeResult rb = runEpisode(orbitScenario(), *b);

    CHECK(ra.stateHash == rb.stateHash);
    CHECK(ra.ticks == rb.ticks);
}

TEST_CASE("reusing one controller instance does not change the result") {
    // Catches a controller that accumulates state across episodes without
    // clearing it in reset() -- which would silently make the second of two
    // ranked runs different from the first.
    auto c = ControllerRegistry::instance().create("seek-far");
    REQUIRE(c);

    const EpisodeResult first  = runEpisode(orbitScenario(), *c);
    const EpisodeResult second = runEpisode(orbitScenario(), *c);
    CHECK(first.stateHash == second.stateHash);
}

TEST_CASE("running in parallel gives the same answers as running alone") {
    // The property the whole eval harness depends on: a world's trajectory must
    // not depend on what else the machine happened to be doing.
    std::vector<Job> jobs;
    for (int i = 0; i < 32; ++i) {
        jobs.push_back(Job{orbitScenario(5'000), "seek-far"});
    }

    const std::vector<EpisodeResult> parallel = runBatch(jobs, 8);

    auto solo = ControllerRegistry::instance().create("seek-far");
    REQUIRE(solo);
    const EpisodeResult reference = runEpisode(orbitScenario(5'000), *solo);

    for (const EpisodeResult& r : parallel) {
        CHECK(r.stateHash == reference.stateHash);
    }
}

TEST_CASE("a world reset returns to the exact starting state") {
    World world(orbitWorld(classic()));
    const std::uint64_t initial = world.hash();

    for (int i = 0; i < 5'000; ++i) world.step(kTickDt);
    CHECK(world.hash() != initial);

    world.reset();
    CHECK(world.hash() == initial);
}

TEST_CASE("the hash actually notices a divergence") {
    // A hash that never changes would make every determinism test above pass
    // vacuously, so check that it is sensitive to a one-tick difference.
    World a(orbitWorld(classic()));
    World b(orbitWorld(classic()));

    for (int i = 0; i < 100; ++i) a.step(kTickDt);
    for (int i = 0; i < 101; ++i) b.step(kTickDt);

    CHECK(a.hash() != b.hash());
}

TEST_CASE("different seeds are still reproducible per seed") {
    Scenario s1 = orbitScenario(3'000);
    Scenario s2 = orbitScenario(3'000);
    s1.seed = 1;
    s2.seed = 2;

    auto c = ControllerRegistry::instance().create("spinburn");
    REQUIRE(c);

    CHECK(runEpisode(s1, *c).stateHash == runEpisode(s1, *c).stateHash);
    CHECK(runEpisode(s2, *c).stateHash == runEpisode(s2, *c).stateHash);
}
