#include "eval/EpisodeRunner.hpp"

#include "core/World.hpp"

namespace rf {

const char* toString(EndReason r) {
    switch (r) {
        case EndReason::Timeout:     return "timeout";
        case EndReason::OutOfBounds: return "out-of-bounds";
    }
    return "?";
}

EpisodeResult runEpisode(const Scenario& scenario, Controller& controller,
                         const std::string& controllerName) {
    WorldConfig cfg = scenario.world;
    cfg.seed        = scenario.seed;

    World world(cfg);
    controller.reset(scenario.seed, cfg);

    EpisodeResult result;
    result.scenario   = scenario.name;
    result.controller = controllerName;
    result.reason     = EndReason::Timeout;

    ControlInput held{};

    for (std::uint32_t i = 0; i < scenario.maxTicks; ++i) {
        if (world.tick() % kControlEvery == 0) {
            held = controller.evaluate(world.observe(0));
        }
        world.setControl(0, held);
        world.step(kTickDt);

        // Checked after the step so a rocket cannot sit exactly on the boundary
        // burning the whole tick budget.
        if (world.anyOutOfBounds()) {
            result.reason = EndReason::OutOfBounds;
            break;
        }
    }

    result.ticks     = static_cast<std::uint32_t>(world.tick());
    result.stateHash = world.hash();
    world.snapshot(result.finalState);
    return result;
}

}  // namespace rf
