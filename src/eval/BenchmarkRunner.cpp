#include "eval/BenchmarkRunner.hpp"

#include "core/World.hpp"

namespace rf {

BenchmarkRun runBenchmark(const BenchmarkSpec& spec) {
    // Zero-g, always -- defaultWorld, never orbitWorld.
    //
    // In an orbit gravity contributes to the measured acceleration and to every
    // velocity error, so what came out would be a score for orbital mechanics
    // the controller was never asked to do. Worse, it would not even be a
    // consistent one: the contribution depends on where in the orbit the run
    // happened to be, so the same controller would score differently for
    // reasons that have nothing to do with it.
    WorldConfig cfg = defaultWorld(spec.rocket);
    cfg.seed        = spec.seed;

    World world(cfg);

    BenchmarkHarness harness(spec.config, spec.seed);
    harness.reset(cfg);

    ControlInput held{};

    // Same shape as SimulationLoop::tickOnce and runEpisode: fixed dt, the same
    // control decimation, the same zero-order hold. If this loop drifted from
    // those, a headless score would stop predicting what the game does.
    for (std::uint32_t i = 0; i < spec.maxTicks; ++i) {
        if (world.tick() % kControlEvery == 0) {
            held = harness.controller().evaluate(world.observe(0));
        }
        world.setControl(0, held);
        world.step(kTickDt);

        // Sampled after the step, so the actuator state is the one that pushed
        // the hull during this tick and the velocity is the result of it.
        harness.sample(world.observe(0), kTickDt);
    }
    harness.finish();

    BenchmarkRun out;
    out.rocket    = std::string(spec.rocket.name.view());
    out.mode      = spec.config.mode;
    out.seed      = spec.seed;
    out.ticks     = static_cast<std::uint32_t>(world.tick());
    out.stateHash = world.hash();
    out.metrics   = harness.monitor().metrics();
    return out;
}

}  // namespace rf
