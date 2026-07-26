# RocketFightV2

A C++ / SFML 3 game and experimentation playground built around a hard separation between
**simulation** and **presentation**.

The point is less "ship a game" and more "have a clean substrate to experiment on": a
deterministic 1 kHz physics core that knows nothing about windows, threads, or SFML; a renderer
that asynchronously *queries* that state instead of driving it; and a headless harness where
rocket controllers — hand-written, generated, or learned — can be run against scenarios in
parallel and compared.

## Goals

1. **1000 Hz fixed-step physics.** Simple, custom, no external physics library. No collisions for
   now — integration, forces, thrust, torque. Collisions are a later layer, and the design must
   not have to be rewritten to accept them.
2. **Model / view separation is structural, not stylistic.** The core simulation compiles and runs
   with zero knowledge of SFML. If a `#include <SFML/...>` ever appears under `src/core/`, the
   design has failed.
3. **Rendering is asynchronous and read-only.** The render loop runs at its own rate (vsync,
   144 Hz, whatever) and never blocks the simulation. It reads the most recently published
   snapshot and draws it. A slow frame must not cost a physics tick.
4. **Control is layered and swappable.** Pilot *intent* (vehicle-agnostic), the *fly-by-wire* that
   resolves intent into thruster commands (vehicle-specific), and end-to-end policies are three
   separate abstractions. A human and an AI actuate a rocket through the same interfaces, and an
   AI may enter at either layer. Neither is privileged.
5. **Headless, parallel, faster than real time.** The same simulation runs with no window, many
   independent worlds at once, so controllers can be evaluated against scenarios and metrics in
   bulk.
6. **Bit-exact determinism on a given machine.** Same binary + same initial state + same input
   sequence => byte-identical trajectory, verifiable by hashing world state. Without this,
   controller comparisons are noise and bug reports are unreproducible.

## Non-goals (for now)

- Collision detection / response
- Networking, and therefore cross-machine bit-exactness
- Asset pipeline, audio, UI toolkit
- Fuel and variable mass — matches are short enough that thrust is free, and constant mass keeps
  `invMass`/`invInertia` constant, which makes early dynamics much easier to reason about
- A third-party ECS (a plain contiguous array of bodies is enough at this scale)

## Architecture

```
                 owns, writes                      reads, never writes
   ┌────────────────────────────┐          ┌──────────────────────────────┐
   │      Simulation thread     │          │   Render / window thread     │
   │                            │          │                              │
   │  World  (pure C++, no SFML)│          │  sf::RenderWindow            │
   │  step(dt = 1ms) @ 1000 Hz  │          │  draws at display rate       │
   │            │               │          │            ▲                 │
   │            ▼               │          │            │                 │
   │       publish(Snapshot) ───┼──────────┼──> StateChannel (lock-free)  │
   │            ▲               │          │            │                 │
   │            │               │          │            ▼                 │
   │     drain(CommandQueue) <──┼──────────┼─── sf::Event -> Command      │
   └────────────────────────────┘          └──────────────────────────────┘
```

Two shared objects, both single-producer / single-consumer, both lock-free:

- **`StateChannel`** — sim → render. A triple buffer. The sim writes a full `Snapshot` into a
  spare slot it owns and atomically publishes it; the renderer atomically acquires the most recent
  published slot and owns it until its next acquire. Neither side ever waits on the other, and
  neither ever reads a half-written state. The three slots are allocated once and reused, so
  publishing at 1000 Hz does not allocate.
- **`CommandQueue`** — input → sim. SFML events are translated into engine-level `Command`s
  (`SetThrottle`, `SetRcs`, `SetGimbal`, `Fire`, `Reset`, …) and consumed at tick boundaries,
  never applied mid-step. Input quantizes to 1 ms, far below perception, and replays stay exact.

### Why snapshots instead of locking the world

A mutex around the world lets the renderer's frame time leak into the simulation's tick budget.
At 1000 Hz that budget is 1 ms; a 4 ms frame stall would eat four ticks. Copying a snapshot is
O(entities) and, at the entity counts this playground will see, far cheaper than the coupling.

### Why there is no render interpolation

Interpolating between snapshots is the standard answer to a 60 Hz simulation feeding a 144 Hz
display. Here the ratio is inverted: the simulation publishes 1000 times a second, so every frame
already has a state that is **at most 1 ms old**, and the frame-to-frame jitter in that staleness
is under a millisecond. There is nothing left to smooth — interpolation would add latency and code
for no visible benefit. Should the tick rate ever drop far below the display rate, the snapshot
carries its `tick` and `time`, which is everything an interpolating renderer would need.

### Layout

```
src/
  core/         Vec2, Body, World, integrator, Command, ControlInput, Intent, Observation,
                Snapshot. Pure C++. No SFML. No threads. No I/O. No wall clock.
  control/      IntentSource, FlyByWire, Controller; LayeredController composing the first two;
                RocketFlyByWire; human + scripted implementations; name→factory registry.
  sync/         StateChannel (triple buffer), CommandQueue (SPSC ring).
  sim/          SimulationLoop: real-time pacing, accumulator, publishing.
  render/       SFML window, follow-camera, Renderer::draw(const Snapshot&).
  app/          main(): wires window + input + sim thread.
  eval/         Scenario, Metrics, episode runner, parallel batch runner. Links core only.
tests/          Determinism hashes, integrator accuracy, channel/queue correctness.
```

`core`, `control` and `eval` have **no link dependency on SFML** — enforced by the CMake target
graph, not by convention. Only `render` and `app` link it.

## The physics model

A 2D rigid body: position and orientation, both integrated.

```cpp
using Real = double;                 // one typedef, so precision is a one-line experiment

struct Body {
    Vec2 pos, vel;
    Real angle{}, angVel{};
    Real invMass{}, invInertia{};    // inverse form: 0 == immovable, no branches
    Vec2 force{};                    // accumulated over the tick, cleared after integrate
    Real torque{};
};

void applyForceAt(Body&, Vec2 f, Vec2 localOffset);   // -> force += f, torque += cross(r, f)
```

Rockets therefore steer physically: thrust applied at a gimballed nozzle offset from the centre of
mass produces torque, and the body has real rotational inertia. This is the reason for choosing
rigid bodies over point masses — retrofitting orientation later would touch every snapshot,
observation, controller, and draw call.

State is `double`. At 1000 Hz an episode accumulates a million integration steps, and long orbital
trajectories are exactly where `float`'s 24-bit mantissa starts to show; the cost is snapshot size,
which is irrelevant at these entity counts. Conversion to `float` happens once, at the boundary
where positions are handed to SFML for drawing.

Integration is **semi-implicit (symplectic) Euler**, which at a 1 ms step is accurate and stable
for thrust-and-gravity dynamics, and behaves far better than explicit Euler over long episodes —
symplectic integrators do not bleed energy out of an orbit the way explicit Euler does. It sits
behind a single `integrate()` function so RK4 or velocity Verlet is a contained swap.

### Environment: zero-g, with optional attractors

The default world is **empty space with no ambient force at all** — pure Newtonian drift, where a
rocket keeps whatever velocity it was last given until something changes it. Gravity is not a
constant buried in the integrator; it is *data*:

```cpp
struct Attractor {          // a dense planet
    Vec2 pos;
    Real mu;                // G·M, the only gravitational parameter that matters
    Real radius;            // for rendering, and for later collision
};

struct WorldConfig {
    std::vector<Attractor> attractors;   // empty  ->  zero-g
    // ...
};
```

Per tick, each body accumulates `Σ mu * r̂ / (|r|² + ε²)` over the attractors. The softening term
`ε` keeps the force finite if a body passes through a singularity, which matters while there are
no collisions to stop it. Zero attractors is the default and costs an empty loop; one or several
dense planets gives orbits, slingshots and unstable trajectories to play with — which is where
this gets interesting for controller experiments later.

Uniform gravity, if it is ever wanted, is one more optional field rather than a different world.

**Units are SI throughout**: metres, seconds, radians, kilograms, newtons. No pixel-space physics —
the renderer converts at the last moment.

**The world is bounded at 1000 km from the origin.** Not a wall: crossing it ends a headless
episode with `OutOfBounds`, and in the windowed game it is drawn as a boundary circle. In zero-g a
rocket that drifts off with a bad controller would otherwise burn the full tick budget travelling
nowhere, and every batch run would be paced by its most useless member.

### The tick

```cpp
constexpr Real     kTickDt      = Real(0.001);   // 1000 Hz, a compile-time constant
constexpr uint32_t kControlEvery = 10;           // controllers run at 100 Hz

// Simulation thread, real-time paced
while (dueTicks-- > 0) {                          // catch-up capped; a stall cannot spiral
    humanSource.consume(commandQueue.drain());    // input applied only at tick boundaries
    if (world.tick() % kControlEvery == 0)
        world.setControl(id, controller.evaluate(world.observe(id)));
    world.step(kTickDt);                          // fixed dt, always; never a frame time
}
channel.publish(world.snapshot());
```

Headless runs call the identical `world.step(kTickDt)` with the identical decimation, in a bare
loop with no pacing, no publishing and no command queue. Same code path, same results.

## Control: three layers, not one

Actuation is split into three abstractions rather than collapsed into one, because *what a pilot
wants* and *which thrusters achieve it* are different problems that change for different reasons.

```
   ┌──────────────┐   Intent    ┌──────────────┐  ControlInput  ┌────────────┐
   │  IntentSource│────────────>│   FlyByWire  │───────────────>│   Body     │
   │  (human, AI) │             │ (per-vehicle)│                │ (physics)  │
   └──────────────┘             └──────────────┘                └────────────┘
          ▲                            ▲                              │
          └────────────────────────────┴──────── Observation ─────────┘

   ┌──────────────────────────────────────────┐  ControlInput
   │  End-to-end AI Controller                │───────────────> (skips both layers)
   └──────────────────────────────────────────┘
```

### Layer 1 — `Intent`: what the pilot wants, vehicle-agnostic

```cpp
struct Intent {
    enum class Mode { Acceleration, Velocity };

    Mode  mode{Mode::Acceleration};
    Vec2  vector{};                  // Acceleration: world-frame direction, |v| <= 1,
                                     //   as a fraction of the vehicle's max acceleration
                                     // Velocity: world-frame target velocity in m/s
    std::optional<Real> facing{};    // desired heading; nullopt = don't care, fly-by-wire is free
    bool  fire{};
};
```

`Intent` mentions no thrusters, no gimbal, no torque. It is expressed entirely in terms a pilot
cares about, so the same intent is valid for any vehicle. `Mode::Velocity` also gives the classic
space-sim "kill relative velocity" for free: target velocity zero.

### Layer 2 — `FlyByWire`: intent → actuators, specific to the vehicle

```cpp
class FlyByWire {
public:
    virtual ~FlyByWire() = default;
    virtual ControlInput resolve(const Intent&, const Observation&) = 0;
    virtual void reset() = 0;
};
```

This is where the vehicle's actual configuration lives — thruster count and placement, gimbal
range, RCS authority, inertia. Achieving a world-frame acceleration with a single gimballed main
engine means *rotating the ship to point at it first*, so the fly-by-wire is a real closed-loop
attitude controller (PD on heading error driving RCS, gimbal for fine correction) with a throttle
law on top. It is stateful, hence `reset()`.

The payoff: give the rocket a wider gimbal or two more RCS thrusters and only the fly-by-wire
changes. The pilot, the HUD, and any intent-level AI are untouched.

### Layer 3 — `Controller`: what the simulation actually talks to

```cpp
struct ControlInput {
    Real throttle{};   // [0, 1]   main engine
    Real gimbal{};     // [-1, 1]  nozzle deflection -> thrust vectoring
    Real rcs{};        // [-1, 1]  attitude thrusters -> pure torque
    bool fire{};
};

class Controller {
public:
    virtual ~Controller() = default;
    virtual ControlInput evaluate(const Observation&) = 0;
    virtual void reset(uint64_t seed, const WorldConfig&) = 0;
};
```

`rcs` exists because gimballed thrust alone cannot rotate a rocket without also accelerating it —
in zero-g that makes the thing unflyable. Real spacecraft carry reaction control thrusters for
exactly this, so the model gets them too: a pure torque, no net force.

The simulation only ever knows `Controller`, and the layering is composition above it:

```cpp
class LayeredController : public Controller {          // human, or an intent-level AI
    ControlInput evaluate(const Observation& o) override {
        return flyByWire_->resolve(source_->intent(o), o);
    }
};

class EndToEndController : public Controller { ... };  // AI straight to ControlInput
```

So an AI can be plugged in at **either** layer, and that choice is a real experiment rather than a
detail: an intent-level AI inherits the fly-by-wire, learns a smaller vehicle-agnostic problem, and
can be dropped into a different rocket unchanged. An end-to-end AI owns attitude control too — a
harder problem with a higher ceiling, since it can exploit dynamics the hand-written fly-by-wire
never will. Both implement the same interface, so they compete on equal terms in the same harness.

A human always drives layer 1. Direct manual control still exists — `DirectHumanController`
implements `Controller` and writes `ControlInput` straight from the keyboard, bypassing the
fly-by-wire — but that is the debugging path and the "fly it raw" path, not the default.

### Two properties that hold across all three

- **`Observation` is a value type** derived from world state — never a pointer into the world.
  Controllers cannot mutate the simulation, cannot see more than the contract allows, and the
  contract stays serializable, which is what will later permit out-of-process or non-C++
  controllers. Static environment data (attractors, vehicle limits) arrives once via `reset`, so
  the per-step observation stays small and allocation-free.
- **Control rate is decimated, not per-tick.** Physics runs at 1000 Hz; the controller chain is
  evaluated every `K` ticks (default `K = 10`, i.e. 100 Hz) and its output is zero-order held in
  between. Running a controller 1000 times a second is both wasteful and unlike any real control
  system. The decimation is a fixed tick count, so determinism is unaffected — and the fly-by-wire's
  PD gains are tuned against that fixed rate.

### Evaluation

```cpp
struct Scenario {                    // initial state + termination
    WorldConfig  world;
    uint64_t     seed;
    uint32_t     maxTicks;
};

struct EpisodeResult {
    uint32_t  ticks;
    Reason    reason;                // Completed / Timeout / OutOfBounds / ...
    uint64_t  stateHash;             // determinism check, free of charge
    Snapshot  finalState;
};

EpisodeResult               run(const Scenario&, Controller&);
std::vector<EpisodeResult>  runBatch(std::span<const Job>, unsigned threads);
```

`runBatch` gives each job its own `World` and its own `Controller` instance on a thread pool —
embarrassingly parallel, nothing shared, no locks in the physics at all. That is the substrate for
"have N agents each write a controller, run them all, compare them".

**Scoring and metrics are deliberately deferred.** Until there is a task worth scoring, an episode
just runs to completion and reports what happened; a `Metrics` accumulator and per-scenario score
function slot in later without disturbing the runner. Getting the *plumbing* right first — many
worlds, in parallel, reproducibly — is the part that is expensive to change afterwards.

Likewise, the mechanism for *authoring* controllers in parallel (in-tree files behind a name
registry, plugins, or an out-of-process protocol) is left open. The `Observation` → `Intent` →
`ControlInput` boundaries are all value types, which is what keeps every one of those options
available; milestone 1 only commits to the registry. An agent writing a controller picks its
layer, adds a file, and registers a name.

## Determinism

Target: **same machine, same binary, bit-exact.** Enough for replays, regression tests, ranking
controllers fairly, and reproducing bugs from a seed. Cross-machine exactness is explicitly not
promised, because it would constrain the math for no benefit here.

Rules, all cheap to hold:

- No `-ffast-math`. `-ffp-contract=off` so optimisation level cannot change results.
- Nothing in `core` reads a wall clock, `rand()`, thread IDs, addresses, or any global.
- All randomness comes from an explicit per-world PRNG seeded from the `Scenario`.
- Fixed iteration order: dense contiguous arrays, stable indices, no hash-map traversal.
- One world is never touched by two threads.

Verified in tests by running a scenario twice and comparing `stateHash`, and by replaying a
recorded command log against a recorded final hash.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
./build/rocketfight            # windowed, human controller
./build/rocketfight_eval       # headless batch, no window
ctest --test-dir build
```

Requires a C++20 compiler and CMake ≥ 3.24.

**SFML 3** is fetched by `FetchContent` at a pinned tag (`3.0.2`) rather than taken from the
distro — Fedora ships 2.6.2, and SFML 3 is a different API (`sf::Event` is a variant queried via
`getIf<>`, `VideoMode` takes a `sf::Vector2u`, scoped enums throughout). Building from source also
keeps the toolchain identical across machines. The first configure pays a one-time SFML build;
afterwards it is cached in `build/_deps`. **doctest** is fetched the same way — header-only and
fast to compile.

Since SFML is built from source, its Linux build dependencies must be present:

```sh
sudo dnf install libX11-devel libXrandr-devel libXcursor-devel libXi-devel \
                 freetype-devel systemd-devel mesa-libGL-devel
```

Audio and networking modules are disabled, so OpenAL is not needed.

## Milestone 1

Prove the whole architecture end to end on deliberately trivial content: **one rocket you can
turn and thrust with the keyboard.** No task, no scoring, no interesting controller — those are
cheap to add once the plumbing underneath them is right, and expensive to retrofit if it isn't.

- [x] CMake skeleton, SFML 3 + doctest via FetchContent, target graph enforcing the SFML-free core
- [x] `core`: `Vec2`, `Body`, semi-implicit Euler, attractors (default: none), 1000 km bound,
      `World::step`, `Snapshot`, state hashing
- [x] `sync`: triple-buffered `StateChannel`, SPSC `CommandQueue`
- [x] `sim`: 1 kHz paced loop on its own thread, publishing snapshots, catch-up capped so a stall
      cannot spiral
- [x] `render`: window, follow-camera with zoom, rocket + thrust plume, boundary circle, and a
      static star grid — in empty space with no reference points, motion is otherwise invisible
- [x] `control`: all three layers — `IntentSource`/`FlyByWire`/`Controller`, `LayeredController`,
      a `HumanIntentSource` (keyboard → `Intent`), a `RocketFlyByWire` (PD attitude hold + throttle
      law), a `DirectHumanController` for raw manual flight, a trivial scripted end-to-end
      controller to prove the interface is not player-shaped, and the name→factory registry
- [x] `eval`: episode runner + parallel `runBatch` over N worlds, reporting ticks/s and hashes,
      no metrics yet
- [x] `tests`: determinism (double-run hash equality), integrator accuracy against a closed-form
      circular orbit, `StateChannel`/`CommandQueue` correctness under real thread contention,
      torque from off-centre force, out-of-bounds termination, and a fly-by-wire convergence test
      (commanded acceleration is achieved within a tolerance, and heading settles without
      oscillating)

Deferred by design: collisions, projectiles, fuel, metrics and scoring, learned controllers,
multiple rockets, render interpolation.

### Controls

A gamepad is the natural device for this design: `Intent` wants an analogue direction *and*
magnitude, which is exactly what a thumbstick is. The keyboard is a fallback and is used only when
no pad is connected — two devices writing the same axes would fight, and whichever was polled last
would win, which looks precisely like a stuck stick.

**Fly-by-wire** (default) — the stick produces an `Intent`, the fly-by-wire flies the ship:

| Gamepad | Keyboard | Effect |
| --- | --- | --- |
| left stick | `W` `A` `S` `D` | desired acceleration: direction *and* fraction of max |
| right stick | arrow keys | desired facing; released (centred) hands attitude back to the fly-by-wire |
| `X` | `X` | kill relative velocity (`Mode::Velocity`, target zero) |

**Direct** — raw `ControlInput`, fly-by-wire bypassed:

| Gamepad | Keyboard | Effect |
| --- | --- | --- |
| right trigger | `W` | main engine throttle |
| left stick X | `A` / `D` | RCS — rotate left / right |
| right stick X | arrows | gimbal the nozzle left / right |

**Always:** `Start`/`Tab` toggles fly-by-wire vs direct, `A`/`Space` fires (wired through, no
projectiles yet), `Back`/`R` resets the world, scroll zooms, `F1` shows the raw input overlay,
`Esc` quits.

Run `./build/rocketfight --world=orbit` for the planet, or `--world=empty` (the default) for
zero-g.

## Status

Milestone 1 is complete and running. Measured on a 20-core machine:

- **Simulation:** a steady 1000.0 Hz at 0.0% of the tick budget, no catch-up resyncs.
- **Rendering:** 120 fps against vsync, snapshot age under 0.01 ms — the renderer is never
  more than a single tick behind, which is why it needs no interpolation.
- **Headless:** 207 M ticks/s across the pool, about 207,000x real time, and identical jobs
  produce identical state hashes.
- **Tests:** 30 cases, 162 assertions, all passing — including a circular orbit that holds its
  radius to under 0.1% over a full revolution, and a triple buffer hammered by two real threads
  without a single torn read.

Next up is whatever you want to experiment with: collisions, projectiles, metrics and scoring, or
the first real controllers.
