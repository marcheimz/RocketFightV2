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
  core/         Vec2, Body, Thruster, RocketSpec, RocketTypes, Rocket, World, integrator,
                Command, ControlInput, Intent, Observation, Snapshot.
                Pure C++. No SFML. No threads. No I/O. No wall clock.
  control/      IntentSource, FlyByWire, Controller; LayeredController composing the first two;
                ThrustAllocator; RocketFlyByWire<RocketT>; human + scripted implementations;
                name→factory registry.
  sync/         StateChannel (triple buffer), CommandQueue (SPSC ring).
  sim/          SimulationLoop: real-time pacing, accumulator, publishing.
  render/       SFML window, follow-camera, Renderer::draw(const Snapshot&).
  app/          main(): wires window + input + sim thread.
  data/         RocketCatalogue: loads rockets/*.json into RocketSpec values.
                The only place in the project that opens a file.
  eval/         Scenario, Metrics, episode runner, parallel batch runner. Links core only.
tests/          Determinism hashes, integrator accuracy, channel/queue correctness.
```

`core`, `control` and `eval` have **no link dependency on SFML** — enforced by the CMake target
graph, not by convention. Only `render` and `app` link it. `core` additionally does no file I/O at
all: parsing rocket JSON lives in `data`, which hands back plain `RocketSpec` values that nothing
downstream can tell came from disk.

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

Rockets therefore steer physically: thrust applied at a nozzle offset from the centre of mass
produces torque, and the body has real rotational inertia. This is the reason for choosing rigid
bodies over point masses — retrofitting orientation later would touch every snapshot, observation,
controller, and draw call.

### Vehicles are thruster layouts, defined in JSON

There is no separate concept of "RCS", no vehicle-shaped code in the world's tick
loop, and no vehicle definitions in the source at all. A rocket is a hull and a list of thrusters,
loaded from [`rockets/*.json`](rockets/) at startup:

```jsonc
{
  // "at" is [along, across] as a fraction of half-length and half-width, so the
  // layout survives resizing the hull. Angles are degrees; a direction of 0 is
  // the force the thruster applies to the hull, pointing forward.
  "name": "classic",
  "mass": 1000.0, "length": 12.0, "width": 3.0,

  "thrusters": [
    { "at": [-1.0, 0.0], "direction_deg": 0.0,
      "max_thrust": 40000.0,
      "min_thrust": 12000.0,     // cannot be throttled below 30%
      "ignition_time": 0.25,     // seconds before it produces anything
      "ramp_up": 120000.0,       // N/s
      "ramp_down": 200000.0,
      "gimbal_deg": 11.5,
      "gimbal_rate_deg": 40.0 }  // the nozzle slews, it does not snap
  ]
}
```

JSON rather than YAML because it is the more common format, and `nlohmann/json` can be told to
accept comments — which removes YAML's one real advantage for hand-authored config. A vehicle
definition nobody can annotate is a vehicle definition nobody maintains.

Attitude control is not postulated, it is *geometry*. Two opposed pairs at nose and tail: fire
diagonally opposite ones and the forces cancel while the torques add, which is a pure couple; fire
the two on the same side and you translate with no net torque. Nothing in the code arranges that —
it falls out of where the thrusters are.

### Actuators have dynamics, and that is the point

A `ControlInput` is a *demand*, not a result. Between asking and receiving:

- **Ignition delay.** A thruster commanded on produces nothing for `ignition_time`. A correction
  decided now does not begin for a quarter of a second on some vehicles.
- **A thrust floor.** Once lit it cannot be throttled below `min_thrust`. It is off, or it is
  somewhere in `[min, max]` — nothing in between, at any price.
- **Ramp rates.** Thrust moves toward its demand at a finite N/s, and decays rather than vanishing.
- **Nozzle slew.** Gimbals travel at `gimbal_rate`, so *reversing* a nozzle takes real time.

All of it is simulation state: integrated every tick, hashed for determinism, and carried into the
snapshot so the renderer draws the plume that exists rather than the one that was requested.

The consequence is that control gets genuinely harder, in ways with names:

- Allocation becomes **partly discrete**. The continuous least-squares answer is often a level the
  vehicle cannot hold, so it is rounded to off-or-floor — and the controller must own that rounding
  rather than let the simulation do it silently.
- Attitude needs a **deadband**. Below some error the smallest available correction is larger than
  the error itself, and chasing it is a limit cycle wearing a control loop's clothes. The
  fly-by-wire computes that limit from the layout and stops inside it.
- Rate profiles need **dead-time compensation**. A gimbal-only vehicle brakes a turn by swinging its
  nozzle all the way across, which takes 1.6 s; a profile that assumes braking is instant commits to
  a rate it cannot stop from and overshoots every time, in both directions.
- Precision is **bounded by physics, not tuning**. `minImpulseDeltaV()` — the thrust floor acting
  over the ignition delay — is the finest velocity change a vehicle can make. For `classic` that is
  3.1 m/s. No controller beats it, so the tests assert against *it* rather than a hopeful constant.

Derived capability is computed from the layout rather than declared, which is what lets one
controller fly all of them:

```cpp
Real maxForwardAccel() const;   // sum of the forward-pointing thrust
Real maxLateralAccel() const;   // the weaker of left and right
Real maxAngAccel() const;       // angular authority from placement alone: zero without RCS
Real maxGimbalAngAccel() const; // ...and the authority that exists only while burning
bool mustPointToThrust() const; // little lateral authority => the nose must lead
```

Four vehicles ship in [`rockets/`](rockets/): `classic` (one gimballed main plus two opposed
pairs), `norcs` (no attitude thrusters at all, so it can only steer while the engine is lit),
`lander` (real side-thrusting authority, so it does not need to point its nose where it is going),
and `interceptor` (twin mains either side of the centreline, making differential throttle a second
source of torque). Adding a fifth means adding a file — no recompilation, and no new control code.

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

### Layer 2 — `FlyByWire`: intent → actuators, built from the spec

```cpp
class RocketFlyByWire final : public FlyByWire {
public:
    explicit RocketFlyByWire(const RocketSpec& spec);   // allocator solved once, here
    ControlInput resolve(const Intent&, const Observation&) override;
};
```

This is where the vehicle's configuration is *used* — thruster placement, gimbal range, angular
authority, thrust floors, ignition delays, inertia. It hardcodes no layout. Whether the nose must
lead, how much angular authority exists, whether that authority survives the engine being shut
down, how long corrections take to arrive, and how tight a deadband the thrust floors permit are
all read from the spec.

That is why one implementation flies a lander, a gimbal-only rocket and a twin-engined interceptor,
and why they still feel like different vehicles — **including rockets added as JSON after this code
was written**.

#### Control allocation

Turning "I want this force and this torque" into thruster levels is the interesting part. Each
thruster contributes a fixed direction of a 3D wrench (force x, force y, torque) scaled by its
level; stack those into a 3×N matrix `B` and the question is to find levels `u` with `B u = w`.
That is generally not exactly solvable, so [`ThrustAllocator`](src/control/ThrustAllocator.hpp)
uses the damped pseudo-inverse:

```
u = Bᵀ (B Bᵀ + λI)⁻¹ w
```

Three properties make this the right tool rather than a clever one:

- **Attitude control falls out for free.** Minimum-norm means asking for pure torque on a symmetric
  layout produces the balanced opposed pair — not because anything balanced it, but because firing
  one thruster alone needs a larger level *and* leaves a residual force the solution is penalised
  for.
- **Unachievable requests degrade instead of lying.** Ask the classic rocket to push sideways and
  it returns approximately nothing, which is the honest answer, rather than a confident wrong one.
  The damping `λ` is what keeps that well-defined when a whole axis of authority is missing.
- **It is layout-agnostic.** `B` is built from the spec, so a new rocket type needs no new control
  code.

Thrusters cannot pull, so negative levels are clamped, and clamping costs real authority — a single
clamped pass delivers only 40% of a symmetric layout's torque. Re-solving on the residual recovers
it, but geometrically, so the solver iterates to convergence rather than the two passes that look
sufficient and quietly stall at 87%.

### Layer 3 — `Controller`: what the simulation actually talks to

```cpp
struct ControlInput {
    std::array<Real, kMaxThrusters> level{};    // [0, 1]  per thruster
    std::array<Real, kMaxThrusters> gimbal{};   // [-1, 1] per thruster, ignored if fixed
    bool fire{};
};

class Controller {
public:
    virtual ~Controller() = default;
    virtual ControlInput evaluate(const Observation&) = 0;
    virtual void reset(uint64_t seed, const WorldConfig&) = 0;
};
```

One entry per thruster, rather than a `throttle`/`gimbal`/`rcs` triple. That triple silently assumed
one gimballed engine plus an abstract torque source — which is precisely the assumption that made a
second engine, or no attitude thrusters at all, impossible to express.

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

Audio and networking modules are disabled, so OpenAL is not needed. **nlohmann/json** is fetched
the same way, for the rocket definitions.

Rockets are looked for in `$ROCKETFIGHT_ROCKETS`, then `./rockets`, then the source directory
configured at build time — so the binaries run from anywhere in the tree with no install step, and
editing a vehicle needs no rebuild.

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
| right trigger | `W` | open every forward-pointing thruster |
| left stick X | `A` / `D` | open whichever thrusters torque that way |
| right stick X | arrows | deflect every gimballed nozzle |

Direct mode still has to know the layout, because "turn left" means firing whichever thrusters
produce a positive torque on *this* vehicle. What makes it direct is that there is no closed loop
and no allocation: the stick opens thruster groups, and whatever the ship then does is your
problem.

**Always:** `Start`/`Tab` toggles fly-by-wire vs direct, `A`/`Space` fires (wired through, no
projectiles yet), `Back`/`R` resets the world, scroll zooms, `F1` shows the raw input overlay,
`Esc` quits.

Run `./build/rocketfight --world=orbit` for the planet, or `--world=empty` (the default) for
zero-g, and `--rocket=classic|norcs|lander|interceptor` to pick the airframe. They fly very
differently, and no controller code changes between them.

## Status

Running, with vehicles as data and actuators that fight back. Measured on a 20-core machine:

- **Simulation:** a steady 1000.0 Hz at 0.0% of the tick budget, no catch-up resyncs.
- **Rendering:** 120 fps against vsync, snapshot age under 0.01 ms — the renderer is never more
  than a single tick behind, which is why it needs no interpolation.
- **Headless:** 116 M ticks/s across the pool, ~116,000x real time, sweeping every controller
  against every rocket in both worlds, with identical jobs producing identical state hashes.
- **Tests:** 58 cases, 2452 assertions, all passing.

Physics, determinism and data-loading are asserted tightly. The closed-loop flying checks are
deliberately loose — they exist to catch "this vehicle cannot be flown at all", not to pin a set of
gains that is expected to be optimised against metrics later.

Next up: brute-force optimisation of the fly-by-wire against the actuator constraints, and then
whatever else you want to experiment with — collisions, projectiles, scoring, learned controllers.
