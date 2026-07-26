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
                Command, ControlInput, Intent, Observation, Snapshot, InlineName.
                Pure C++. No SFML. No threads. No I/O. No wall clock.
  control/      IntentSource, FlyByWire, Controller; LayeredController composing the first two;
                ThrustAllocator; RocketFlyByWire<RocketT>; human + scripted implementations;
                name→factory registry; the deterministic benchmark sequence and its meter.
  sync/         StateChannel (triple buffer), CommandQueue (SPSC ring).
  sim/          SimulationLoop: real-time pacing, accumulator, publishing.
  render/       SFML window, follow-camera, actuator panel, Renderer::draw(const Snapshot&).
  app/          main(): wires window + input + sim thread.
  data/         RocketCatalogue: loads rockets/*.json into RocketSpec values.
                SubmissionCatalogue: finds submitted .so files so they can be
                flown by hand. The layer that is allowed to open a file.
  eval/         Scenario, episode runner, parallel batch runner, benchmark runner + JSON,
                and rocketfight_runner: one benchmark job in one process, which is the
                only place a submitted library is ever loaded.
  server/       Sandbox (fork + rlimits + wall-clock SIGKILL), Compile, Evaluator,
                Leaderboard. Linux-only and unapologetically so.
tests/          Determinism hashes, integrator accuracy, channel/queue correctness,
                and the submission boundary's failure paths.
include/        rocketfight/flybywire_abi.h -- the public submission contract. Its own
                include root, so a submission cannot reach src/ by accident.
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
    { "name": "main",            // a label, for the HUD -- never read by physics
      "at": [-1.0, 0.0], "direction_deg": 0.0,
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

Each thruster carries a `name` for the same reason: once the actuator panel shows sixteen bars,
"nose-left is still lighting" is a sentence and "index 3 is still lighting" is a lookup. Nothing in
the physics, the allocator or the fly-by-wire reads it, and nothing may start to — the moment
behaviour depends on a name, layouts stop being interchangeable and the JSON stops being data.
Omit it and a thruster is called `thruster-N` for its own index, so an unannotated file still
loads and its labels still match its subscripts. The name is a fixed inline `char` buffer, not a
`std::string`, because a `RocketSpec` is copied into every snapshot a thousand times a second;
`static_assert(std::is_trivially_copyable_v<...>)` on `Thruster`, `RocketSpec`, `ControlInput` and
`RocketView` is what stops that being re-learned the hard way, since the failure mode of adding an
owning member is not a compile error but a silent allocation on the publish path.

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

### Watching the actuators

The world view draws plumes from actual thrust, which means the most interesting thing the
simulation models is the one thing it cannot show: between commanding a thruster and seeing it,
there is simply nothing on screen. A quarter-second of ignition delay looks identical to a
controller that decided to do nothing.

So there is a side panel with one column per thruster, and it draws *both* numbers:

- the **demand**, `ctrl.level[i] * maxThrust`, as an outline;
- the **actual**, `actuators[i].thrust`, as a filled bar inside it;
- a distinct cool colour while a thruster is **igniting** — commanded on, `ignitionTimer > 0`, not
  yet `lit` — because at that moment the honest fill height is zero and a warmer colour would read
  as "a little bit of thrust";
- a tick at `minThrust`, since a lit thruster is somewhere in `[min, max]` and nowhere below, so a
  bar resting on that line is the vehicle refusing rather than the controller being timid.

The outline jumps the instant the fly-by-wire commits and the fill crawls up behind it. That gap is
the dead time every one of the control problems above is really about, and it is the reason the
panel is a first-class part of the renderer rather than a debug overlay.

Each bar is normalised against its own thruster's maximum, so a 2.5 kN attitude puff at full
deflection reads as full — which is what "is this nozzle saturated?" means. The absolute figure is
in the label, next to the name, so the two questions stay separable.

Thrust is only half of what an actuator lags on, so under each bar — for the nozzles that can
actually move — is a horizontal strip spanning `[-maxGimbal, +maxGimbal]` with a marked centre,
saying the same two things in the same grammar: **the commanded deflection as an outline growing
out of zero, the angle the nozzle has actually reached as a fill inside it.** A gimbal slews rather
than snaps, so a fill trailing its outline is that travel happening, and a fill still on the
opposite side of centre is a reversal in progress — the vehicle being turned the wrong way *right
now* by a nozzle that has been told to turn it the other way. On `norcs`, whose only means of
steering is that nozzle, going stop to stop takes 1.6 s; without the strip, a second and a half of
committed-but-not-yet-arrived attitude command looks exactly like a controller doing nothing, which
is the same lie the thrust bars exist to stop the plumes telling.

Only vehicles that have a gimbal somewhere get the row at all — no empty band on a rocket that
cannot steer that way — and within the row a fixed nozzle simply has no strip rather than one
pinned at centre, which would read as "aimed straight" instead of "cannot move". Mixed layouts are
the normal case: `classic` and `lander` gimbal one main and fly four fixed puffers, `interceptor`
gimbals two, `norcs` has nothing else. The strip is drawn whether or not the engine is lit, because
a nozzle has an angle either way and swinging it before ignition is a real tactic — though the
torque it buys is proportional to throttle, so a deflected nozzle on a cold engine is doing nothing
yet, and the bar above it says so.

Everything it needs is already in the snapshot: `RocketView` carries `spec`, `ctrl` and
`actuators`. The panel reaches into nothing.

### The vectors on the ship

Three lines leave the hull in the world view, and between them they say what the ship is doing and
what it was asked to do:

- **velocity**, green, drawn as *where you will be in one second* — so its length is a distance that
  can be read against the grid rather than an arrow of arbitrary scale;
- **net thrust**, orange, the vector sum of every firing nozzle. On a symmetric attitude burn it is
  correctly zero: the ship is rotating, not accelerating, and the HUD should say so;
- **the command**, dashed, in whichever of those two hues matches the current intent mode.

The dash is the panel's grammar carried out into the world: demand is a ghost, reality is solid. It
keeps the *hue* of the line it is being compared with, because it is the same quantity asked for
rather than achieved — a third colour would read as a third thing. A cap across its tip keeps the
endpoint readable in the good case, where the two lie on top of each other.

Only the line matching the mode is drawn, never both: the two carry different units, and a length in
one is not a length in the other. In acceleration mode `Intent::vector` is already a fraction of the
vehicle's maximum, so it is multiplied by `maxForwardAccel()` and put through the **same** length
mapping the achieved thrust just went through. That sharing is the entire point — two arrows on
different scales would only ever say "some" and "some". In velocity mode the target gets the same
one-second convention as the actual velocity, so the distance between the two tips is the tracking
error in m/s, read straight off the screen.

The interesting frame is the one where the ship is still slewing onto a new command: the dashed
vector is at full length and there is no solid one at all, because the main engine is a quarter of a
second into its ignition delay. That is the same dead time the actuator panel exists to show, said
once more in the units the pilot actually commanded in.

The renderer draws only from the `Snapshot`, and an `Intent` belongs to the controller, which lives
on the simulation thread. So the command travels exactly the way `SimStats` does: a plain field that
`World::snapshot` leaves zeroed and `SimulationLoop::publish` fills in from
`Controller::lastIntent()`. They are the same shape of thing — a fact about whoever is flying, and
the world has no controller any more than it has a clock. It carries a `valid` flag because
`lastIntent()` legitimately returns null: `DirectHumanController` does not think in terms of intent
at all, and a zeroed `Intent` is indistinguishable from a real command for zero acceleration. Flying
direct therefore draws nothing, rather than an arrow nobody asked for.

This costs the headless path nothing (it never fills the field) and determinism nothing (the state
hash covers world state; presentation fields are not part of it). The `static_assert` on
`std::is_trivially_copyable_v` sits beside the one on `RocketView` for the same reason — `Intent`
holds a `std::optional<Real>`, it qualifies today, and the failure mode of a member that stops
qualifying is a silent allocation on a path that runs a thousand times a second.

It also comes for free while the benchmark is running, because the benchmark is not a special case:
it is an `IntentSource` feeding the same active controller, so its scripted commands appear on the
ship like anyone else's, and the step changes it makes are visible as they happen.

### The window

The window is resizable, and resizing changes **how much world is visible, not how big things
look**: the camera holds metres-per-pixel across the change and lets the framed width follow the
new pixel width. Zoom stays something the wheel does on purpose. The aspect ratio is derived from
the live viewport, so nothing is ever stretched.

Screen-space overlays build their own view from `window.getSize()` rather than using
`getDefaultView()`. SFML fixes the default view at window *creation* and its resize handler only
recomputes the current view's viewport — so after a drag, the default view still describes the old
pixel size and everything drawn through it comes out stretched and mispositioned. That is a bug
that only appears once somebody resizes, which is exactly the kind that ships.

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

## Benchmarking a fly-by-wire

The eventual shape of this is a submission server: an agent submits a `FlyByWire` implementation,
the server compiles that one file, runs it through a fixed benchmark, and ranks it. This is the
benchmark half.

### The task: a scripted pilot

`BenchmarkIntentSource` is an `IntentSource` — layer 1, the same slot the gamepad occupies. That is
the whole design: what is being measured is the fly-by-wire, so whatever sits above it has to be
interchangeable with a human, and the benchmark is exactly a pilot who never gets bored.

It emits a step sequence:

- **Acceleration mode.** Direction uniform on the circle, magnitude uniform in [0.3, 1.0] of the
  vehicle's maximum. `Intent`'s acceleration vector is *already* normalised as a fraction of max, so
  the band is vehicle-agnostic without the source ever asking what vehicle it is flying.
- **Velocity mode.** Direction uniform on the circle, magnitude uniform in [0, 100] m/s. These are
  **absolute** targets, not scaled to the airframe: in zero-g there is no maximum speed to scale
  against, so 100 m/s is simply the band being tested — `kBenchmarkMaxSpeed`, one constant, also
  used by the pilot's stick so hand-flying and the benchmark are the same task.
- Each value is **held for an interval uniform in [1, 5] s**, then resampled.
- **The two modes are never interleaved in one run.** Their commanded quantities have different
  units; a tracking error integrated across both would be adding metres to m/s.
- **`facing` stays empty throughout.** Where to point is the fly-by-wire's own judgement — most of
  the job on `norcs`, barely any of it on `lander` — and this scores that judgement rather than the
  ability to obey an attitude order.

**The sequence depends on the seed alone.** Not on the vehicle, not on the rocket's state, not on
what the controller did with the last command, and not on which tick the pilot armed it. It uses the
project's own `Rng` (splitmix64, explicitly seeded), quantises every interval boundary to whole
ticks, keeps its own tick counter rather than reading `obs.tick`, and takes exactly three draws per
command in either mode — so an acceleration run and a velocity run from one seed step at the same
instants in the same directions. Two rockets, or two submitted controllers, face a byte-identical
task, and the tests assert that exactly rather than approximately.

### What is measured, and why those four

| Metric | Units | Why |
| --- | --- | --- |
| **Tracking error** | ∫\|commanded − actual\| dt | The task itself. Reported as an integral *and* a mean, since the integral grows with run length. |
| **Settling time** | s, mean and worst | A controller that reaches the right answer eventually is not the same as one that reaches it. Measured per step change, from the resample to the last moment the error was outside tolerance. |
| **Impulse used** | N·s | Two controllers that track equally well are not equally good if one burns twice the propellant. |
| **Attitude wander** | rad | Integrated \|angular velocity\|, so it is total angle swept rather than net rotation — spinning one way and back has done the work twice. Rotation nobody asked for is wasted authority, and in a fight it is a liability. |

Two things this gets deliberately right:

- **It runs in the zero-g world** (`defaultWorld`, never `orbitWorld`). In an orbit gravity
  contributes to every measured acceleration and to every velocity error, so the output would be
  partly a score for orbital mechanics the controller was never asked to do — and not even a
  consistent one, since the contribution depends on where in the orbit the run happened to be.
- **Actual acceleration comes from the vehicle, not from differencing positions.** `ThrusterState`
  carries the thrust actually being produced and the angle the nozzle has actually reached, which is
  precisely what `Rocket::applyControls` turns into force. Summing that wrench measures the vehicle;
  differencing a trajectory would measure the integrator, and would fold in anything else that was
  pushing.

An unsettled step is charged the whole hold interval rather than dropped, which is a floor on the
truth — dropping it would let a controller that *never* settles outscore one that settles slowly, by
contributing no samples at all. `settled_fraction` is reported next to it so the two are separable.

**The components are the output.** There is a `score`, but it is a weighted sum of published
components with the weights shipped beside them, so an archive of results can be re-ranked without
re-running anything. The weights are a placeholder and are meant to be overruled once there are real
submissions to look at.

### Where it lives, and why

`control/Benchmark.{hpp,cpp}` holds the sequence and the meter; `eval/BenchmarkRunner.{hpp,cpp}`
holds the runner.

The split is forced by the target graph, and keeping it honest was the point. The benchmark has to be
drivable from **`rf_sim`** (live, in the window) *and* **`rf_eval`** (headless), and `rf_sim` links
`rf_control` but not `rf_eval`. Putting the generator in `control/`, next to `ScriptedControllers`,
reaches both without a new edge — and without `rf_sim` picking up, through `rf_eval → rf_data`, a
dependency on file I/O it has no business having. What stayed in `eval/` is the half that builds a
`World` and loops it, which is the half `rf_sim` already has its own version of and must not gain a
second copy of.

Nothing was added to `core/` except two `InputButton` names, which is where the input vocabulary
already lives.

### Headless: `rocketfight_bench`

```sh
./build/rocketfight_bench --rocket=classic --mode=accel --seconds=60 --seed=7
./build/rocketfight_bench --all --mode=velocity --compact
```

| Flag | Default | Effect |
| --- | --- | --- |
| `--rocket=NAME` | first in the catalogue | which airframe |
| `--all` | off | sweep every rocket in the catalogue |
| `--mode=accel\|velocity` | `accel` | which of the two experiments |
| `--seconds=N` | 60 | simulated seconds per run; refused above 300, see below |
| `--seed=N` | fixed default | the sequence; accepts `0x…` |
| `--compact` | off | one line instead of indented |

#### Why the run length is capped

The benchmark loop deliberately does **not** check `World::anyOutOfBounds()`, unlike `runEpisode`.
Every run integrates over exactly the same number of ticks however far the vehicle wandered — and
it has to, because acceleration mode is a random walk in velocity, so a stronger vehicle travels
further under the identical normalised command sequence. Measured, the interceptor covers about
twice what `classic` does. Terminating on the bound would hand it a shorter run, and
time-integrated metrics over unequal windows are not comparable. Ranking submissions requires that
every one of them gets the same window.

Drift is inert here in a way it would not be elsewhere: the benchmark runs in `defaultWorld`, which
has no attractors, so position feeds no force, and no metric or controller reads it. The run is
byte-for-byte what it would have been at the origin.

The exemption has a shelf life, though, and it is not the one you would guess. Drift is close to
ballistic rather than a random walk, because **no rocket in `rockets/` has a retro thruster**: a
command with a rearward component produces nothing rearward, leaving a standing bias along the
nose. It is starkest on `lander`, which never rotates, and whose drift barely varies with seed at
all. Measured across every rocket and 40 seeds, vehicles start crossing the 1000 km world bound
from around **450 s** — earliest observed, the interceptor at 449 s.

At the 60 s default the margin is roughly 17x, with the worst case reaching 6% of the bound and
zero crossings in 1600 runs. So 300 s is not a physics limit; it is the edge of the validated
envelope, and `--seconds` refuses to go past it rather than silently producing a run whose vehicles
sit outside a boundary the rest of the engine treats as real.

JSON on stdout, and nothing else:

```jsonc
{
  "benchmark": "flybywire-tracking",
  "version": 1,
  "world": "zero-g",
  "config": {
    "mode": "acceleration", "seed": 7, "seconds": 60.0,
    "hold_seconds": [1.0, 5.0],
    "accel_fraction_of_max": [0.3, 1.0],
    "velocity_max_mps": 100.0,
    "accel_settle_fraction": 0.1,
    "velocity_settle_tolerance_mps": 5.0,
    "control_hz": 100.0
  },
  "score_weights": { "tracking": 1.0, "settling": 1.0, "impulse": 1e-05, "wander": 1.0 },
  "runs": [
    {
      "rocket": "classic", "mode": "acceleration", "seed": 7,
      "ticks": 60000, "seconds": 60.0,
      // Determinism, provable rather than promised: same binary, same seed,
      // same vehicle => same hash. Hex string, because a 64-bit integer does
      // not survive a JSON parser whose number type is a double.
      "state_hash": "0x160d0a56702ae11c",
      "metrics": {
        "tracking_error_integral": 647.27, "tracking_error_mean": 10.79,
        "settling_time_mean_s": 1.40,     "settling_time_worst_s": 2.08,
        "settled_fraction": 0.79,         "steps": 19, "steps_settled": 15,
        "impulse_ns": 1033804.4,          "mean_thrust_n": 17230.1,
        "attitude_wander_rad": 36.37,     "mean_ang_rate_rad_s": 0.606
      },
      "score": 12.97
    }
  ]
}
```

A bad flag or an unknown rocket prints `{"error": "..."}` and exits non-zero — still JSON, because
the thing reading this is a program.

### Live, in the window

The same benchmark runs in the game, driven from the pad. `B` arms it; while it runs it **replaces**
the pilot, and the HUD says so in as many words, because a dead stick with no explanation looks
exactly like a controller that has crashed. `Y` toggles acceleration versus velocity mode, and that
applies to the human too: in velocity mode the left stick commands a target velocity scaled to the
same 100 m/s band. `X` / `KillVelocity` is untouched and stays a momentary "null my velocity" in
either mode.

Live metrics reach the renderer as relaxed atomics on `SimulationLoop` rather than in the `Snapshot`
or through a third channel: the snapshot is the *world's* state and the benchmark is not part of the
world, and a reader that sees one field a control step out of date is showing a HUD nobody can
perceive is stale. The input layering is intact — `InputTranslator` gained two buttons and still
reports only that a button went down.

## The submission server

The other half: an agent submits a `FlyByWire`, the server builds it, runs the benchmark above
against every vehicle in both modes, checks that it reproduces, and ranks it.

> **[`COMPETITION.md`](COMPETITION.md) is the short form** — the whole contract in about 190 lines,
> sized so an agent can hold all of it in context at once and still have room to think. Start there.
>
> **[`SUBMISSIONS.md`](SUBMISSIONS.md) is the full reference.** The ABI function by function, units
> and sign conventions, the manifest, runnable build commands for C, C++ and Rust, the real resource
> limits, the determinism rule, and a complete worked example — written so that nobody has to read
> this source tree to make a submission work.

```sh
./build/rocketfight_server --port=8080 --data=server-data
curl -X POST http://localhost:8080/api/submissions -F artifact=@flybywire.c -F manifest=@manifest.json
```

### The boundary is a C ABI, and the isolation is a process

[`include/rocketfight/flybywire_abi.h`](include/rocketfight/flybywire_abi.h) is five C functions and
four POD structs. Nothing of the host's C++ crosses it — no classes, no `std::` types, no
exceptions, no allocator, no RTTI — which is what makes a Rust or a hand-written-C submission
exactly as first-class as a C++ one, and it is checked rather than claimed: `simple_c` is *built as
C99* by this repo's CMake, and would stop compiling the moment the ABI quietly became a C++ plugin
interface.

The observation deliberately carries **actual** actuator state — the thrust being produced, the
angle the nozzle has reached — and not the last command. That single choice is what makes the
ignition-delay and slew problem solvable at all; a controller that only knows what it asked for
re-commands corrections already on their way and overshoots every time.

**One job, one process.** `rocketfight_runner` loads one library, runs one configuration, prints one
JSON object, exits. A submission's `rf_resolve` can spin forever, recurse until the stack is gone,
or free a pointer it never allocated, and none of those can be contained from inside the process
they happen in — there is no portable way to kill a hung function call, and a corrupted heap is
corrupted for everybody sharing it. So the only honest containment is a process boundary, and
everything about supervision belongs to the parent: the wall-clock deadline, the rlimits, the
`SIGKILL` to the whole process group, the scratch directory, the scrubbed environment.

It costs a fork per configuration — a few hundred microseconds against a 20 ms run — and buys
per-job attribution ("crashed on `norcs` in velocity mode", not "crashed") and a *stronger*
determinism check, since the two runs being compared live in two different address spaces.

### Say what the sandbox is, and what it is not

`setrlimit` plus a kill timer is **hardening against buggy and casually hostile code**, not a
security boundary against a determined attacker. Arbitrary native code execution is precisely what a
`.so` submission *is*, and the only real answers to that are a container, a VM, or a seccomp policy
far stricter than anything here. There is no filesystem confinement and no syscall filtering; the
network namespace is best-effort and is skipped where the kernel does not permit it. The server
refuses to run as root, because every limit it applies is one root can lift.

That is stated in the header of [`src/server/Sandbox.cpp`](src/server/Sandbox.cpp), in
`SUBMISSIONS.md`, and on the server's own startup banner. Implying safety that is not there is worse
than having none.

### Failure is a result, never an error

Every way a submission can be wrong gets a row with a reason and does not stop the other twenty-three
configurations from running: won't compile, isn't an ELF object, missing `rf_resolve`, wrong
`rf_abi_version`, `rf_init` declines, segfault, infinite loop, endless output, an exception that
escaped. NaN and ±inf are replaced with zero — not clamped to the range limit, which would turn
"computed nonsense" into "commanded hard over", a command the submission never meant and one that
would score.

The two limits catch different things and both are needed. A busy-spinning submission burns CPU and
dies to `RLIMIT_CPU` at about 5 s; one that merely *blocks* consumes no CPU and no memory and sails
past every rlimit there is, and only the parent's wall clock ends it, at 10.00 s. Both are measured
in [`tests/test_submission.cpp`](tests/test_submission.cpp) rather than asserted here.

### Determinism is enforced, not requested

The host re-runs the first seed of every `(rocket, mode)` pair in a second process and compares the
state hash the benchmark already emits. A submission that disagrees with itself is flagged
`nondeterministic`, published with its numbers so the submitter can see them, and **never ranked** —
one entry that will not happen the same way twice makes every comparison on the board meaningless,
including its own.

### Scoring is a policy file, not a constant

`weights.json` is created on first start and never overwritten. Because every per-run component is
archived, re-weighting the board is a re-read of stored results rather than a re-run of anything,
and the weights are published on every leaderboard response so anyone can recompute the ranking
themselves. They are a guess made before any real submission existed and are meant to be overruled.

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
./build/rocketfight_bench      # headless fly-by-wire benchmark, JSON on stdout
./build/rocketfight_runner     # one benchmark job, one process -- what the server forks
./build/rocketfight_server     # the submission server; see SUBMISSIONS.md
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
the same way, for the rocket definitions, and **cpp-httplib** for the submission server -- header
only, with OpenSSL, zlib and Brotli all switched off, since the server offers one JSON endpoint and
one static page and every optional feature is another thing to build on a machine that only wanted
to rank rockets.

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
- [x] `render`: resizable window, follow-camera with zoom, rocket + thrust plume, boundary circle,
      a static star grid — in empty space with no reference points, motion is otherwise invisible —
      a per-thruster actuator panel showing demand against reality, and velocity / net thrust /
      commanded vectors on the hull saying the same thing in the pilot's own units
- [x] `control`: all three layers — `IntentSource`/`FlyByWire`/`Controller`, `LayeredController`,
      a `HumanIntentSource` (keyboard → `Intent`), a `RocketFlyByWire` (PD attitude hold + throttle
      law), a `DirectHumanController` for raw manual flight, a trivial scripted end-to-end
      controller to prove the interface is not player-shaped, and the name→factory registry
- [x] `eval`: episode runner + parallel `runBatch` over N worlds, reporting ticks/s and hashes, plus
      a deterministic fly-by-wire benchmark (`rocketfight_bench`) reporting tracking error, settling
      time, impulse and attitude wander as JSON
- [x] `server`: a submission service — a plain C ABI (`include/rocketfight/flybywire_abi.h`), a
      `dlopen` adapter, a fork/rlimit/wall-clock-SIGKILL sandbox, an in-sandbox compiler for C and
      C++ source, one supervised process per `(rocket, mode, seed)` job, a cross-process determinism
      gate, and a persisted leaderboard with data-driven weights. Contract in
      [`SUBMISSIONS.md`](SUBMISSIONS.md)
- [x] `tests`: determinism (double-run hash equality), integrator accuracy against a closed-form
      circular orbit, `StateChannel`/`CommandQueue` correctness under real thread contention,
      torque from off-centre force, out-of-bounds termination, a fly-by-wire convergence test
      (commanded acceleration is achieved within a tolerance, and heading settles without
      oscillating), thruster-name parsing and defaulting, changing vehicle across the command queue
      on a live simulation thread, the benchmark's own reproducibility — same seed giving a
      bit-identical command sequence and state hash, different seeds giving different ones, and the
      sequence being provably independent of the vehicle — and the submission boundary's failure
      paths: every way a library can fail to load, a wall-clock kill of a child that blocks, an
      rlimit kill of one that spins, a segfault, an output flood, and environment scrubbing

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
| left stick | `W` `A` `S` `D` | in acceleration mode: desired acceleration, direction *and* fraction of max. In velocity mode: desired velocity, up to 100 m/s at full deflection |
| right stick | arrow keys | desired facing; released (centred) hands attitude back to the fly-by-wire |
| `X` | `X` | kill relative velocity (`Mode::Velocity`, target zero). Momentary, and it wins in either mode |
| `Y` | `V` | acceleration ⟷ velocity mode, for the pilot *and* the benchmark |
| `B` | `B` | arm/disarm the benchmark. While armed it replaces the pilot entirely |

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
projectiles yet), `Back`/`R` resets the world, `LB`/`RB` or `[`/`]` cycles the vehicle, the
**D-pad** or `,`/`.` cycles the fly-by-wire, scroll zooms, `F1` shows the raw input overlay,
`Esc` quits.

### Flying a submission by hand

The D-pad steps through the built-in fly-by-wire and every submission found in `server-data`
(`--submissions=PATH` to look elsewhere; a plain directory of loose `.so` files works too). The
built-in is always entry zero — whatever a submission turns out to do, there has to be a way back
to something known to work. The HUD names whichever is flying.

This composes with everything else, which is the point of layer 2 being an interface rather than a
class: select a submission, press `B`, and the benchmark scores **that** submission live, on the
same seeded sequence the leaderboard ranks it on. Cycle the vehicle underneath it and the
submission is re-initialised against the new airframe — `rf_init` takes the spec, so flying `norcs`
with `classic`'s thruster count would be a real bug rather than a cosmetic one.

> **The game cannot sandbox a submission.** The server runs each evaluation in a separate,
> supervised process precisely because a submission can hang or crash; `dlopen` here puts it inside
> the game's own process, where no wall-clock kill can reach it. A buggy submission takes the window
> down with it. That is an acceptable trade for flying your own work, and `examples/submissions/hostile_spin`
> is sitting right there — do not load it and expect to get the window back.

Run `./build/rocketfight --world=orbit` for the planet, or `--world=empty` (the default) for
zero-g, and `--rocket=classic|norcs|lander|interceptor` to pick the airframe you start in. They fly
very differently, and no controller code changes between them.

#### Changing vehicle at runtime

Swapping airframes means a different thruster count, a different inertia and a different set of
live actuators, so it is not a mutation of the world — it is a new one. The `World` lives on the
simulation thread and nothing on the render side may reach it, which is the invariant the whole
snapshot design exists to protect.

So the app, which is the layer that owns the catalogue, builds one `WorldConfig` per rocket up
front and hands the list to the `SimulationLoop` before the thread starts. `NextVehicle` and
`PrevVehicle` then travel down the ordinary command queue like any other button, and the
simulation thread rebuilds its `World` at a tick boundary and re-`reset()`s both controllers
against the new config — the fly-by-wire's deadband, dead-time compensation and allocator are all
derived from the spec, so skipping that would fly the new airframe with the old one's numbers.

The two threads still share exactly two lock-free objects. There is no mutex and no third channel:
the roster is written before the thread exists and read only by it, and the render side learns
which vehicle it is flying the only way it is allowed to — by reading `spec.name` out of the
published snapshot, which is also what tells it to re-frame the camera.

`rf_sim` sees a plain `std::vector<WorldConfig>` and still has no idea that rockets are loaded from
files, so it gains no dependency on `rf_data`.

## Status

Running, with vehicles as data, actuators that fight back, and a panel that shows them doing it.
Measured on a 20-core machine:

- **Simulation:** a steady 1000.0 Hz at 0.0% of the tick budget, no catch-up resyncs.
- **Rendering:** 120 fps against vsync, snapshot age under 0.01 ms — the renderer is never more
  than a single tick behind, which is why it needs no interpolation.
- **Headless:** 92 M ticks/s across the pool, ~92,000x real time, sweeping every controller
  against every rocket in both worlds, with identical jobs producing identical state hashes.
- **Benchmark:** a 60 s run per vehicle in about 20 ms single-threaded, byte-identical across runs.
- **Submission server:** a full evaluation — 4 rockets x 2 modes x 3 seeds, plus 8 cross-process
  determinism repeats — in about 0.5 s wall for a well-behaved submission. HTTP stayed at 200 in
  ~0.9 ms throughout a run in which one submission hung on all 24 of its jobs.
- **Tests:** 95 cases, 17012 assertions, all passing.

Physics, determinism and data-loading are asserted tightly. The closed-loop flying checks are
deliberately loose — they exist to catch "this vehicle cannot be flown at all", not to pin a set of
gains that is expected to be optimised against metrics later.

Next up: real submissions to retune the weights against, brute-force optimisation of the fly-by-wire
against `rocketfight_bench`, and then whatever else you want to experiment with — collisions,
projectiles, learned controllers.
