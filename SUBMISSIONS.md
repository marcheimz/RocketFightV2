# Submitting a fly-by-wire

You write one shared object that turns *what the pilot wants* into *what each thruster does*. The
server compiles it if you sent source, runs it against every vehicle in
[`rockets/`](rockets/) in both benchmark modes, checks that it reproduces, and ranks it.

Everything you need is in one header — [`include/rocketfight/flybywire_abi.h`](include/rocketfight/flybywire_abi.h).
Nothing else in this repository is part of the contract, and you should not read it to make a
submission work. This document is the rest.

**Table of contents**

- [The shortest possible submission](#the-shortest-possible-submission)
- [The ABI](#the-abi)
- [The two things that are easy to get wrong](#the-two-things-that-are-easy-to-get-wrong)
- [The manifest](#the-manifest)
- [Building the shared object](#building-the-shared-object)
- [Submitting](#submitting)
- [What is measured, and how it is scored](#what-is-measured-and-how-it-is-scored)
- [The limits](#the-limits)
- [Determinism](#determinism)
- [The sandbox is hardening, not a security boundary](#the-sandbox-is-hardening-not-a-security-boundary)
- [A complete worked example](#a-complete-worked-example)
- [Testing locally before you submit](#testing-locally-before-you-submit)
- [Failure modes and what they look like](#failure-modes-and-what-they-look-like)

---

## The shortest possible submission

This compiles, loads, flies (badly), and gets a row on the board. Everything after this section is
detail.

```c
#include "rocketfight/flybywire_abi.h"

RF_EXPORT int  rf_abi_version(void)               { return RF_ABI_VERSION; }
RF_EXPORT int  rf_init(const RfRocketSpec* spec)  { (void)spec; return 0; }
RF_EXPORT void rf_reset(void)                     {}
RF_EXPORT void rf_shutdown(void)                  {}

RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    (void)intent;
    /* Everything off. A valid answer, and a bad one. */
    (void)obs; (void)out;
}
```

```sh
cc -std=c99 -O2 -ffp-contract=off -fPIC -shared -fvisibility=hidden \
   -I include flybywire.c -o flybywire.so -lm

curl -X POST http://HOST:PORT/api/submissions \
     -F artifact=@flybywire.c \
     -F name="my-controller" \
     -F description="What it does and why."
```

---

## The ABI

Five C functions. No classes, no `std::` types, no exceptions, no allocator, no callbacks, no
ownership transfer across the boundary. That is what makes a C, C++, Rust, Zig or
hand-written-assembly submission equally first-class.

### Call order

Per process, single-threaded, never concurrent:

```
rf_abi_version()      checked first, before the library is trusted with anything else
rf_init(spec)         once. Nonzero return declines the vehicle and fails that episode.
rf_reset()            at least once, before the first resolve
rf_resolve(...) * N   N = 6000 for a 60-second run: 100 Hz of simulated time
rf_shutdown()         once, then the process exits
```

The host runs **one episode per process**, so global state is fine and there is no handle to thread
through. `rf_reset()` means "forget everything except the spec".

### `rf_abi_version`

```c
RF_EXPORT int rf_abi_version(void);
```

Must return `RF_ABI_VERSION` (currently **1**). Called before anything else, so it must not depend
on `rf_init` having run. A mismatch is a hard refusal: a struct-layout mismatch does not announce
itself, it reads plausible garbage out of the middle of an observation and looks like a controller
that flies badly.

### `rf_init`

```c
RF_EXPORT int rf_init(const RfRocketSpec* spec);
```

Fit the controller to a vehicle. **Solve your allocator here**, not in `rf_resolve`: this runs once,
`rf_resolve` runs 6000 times per run and 144,000 times per submission.

Return `0` on success. Nonzero declines the vehicle — the host records that episode as a failure,
charges the failure penalty, and moves on to the next rocket. Declining is legitimate, but it is
expensive; see [scoring](#what-is-measured-and-how-it-is-scored).

`spec` points to host memory valid **only for this call**. Copy what you want to keep.

```c
typedef struct RfRocketSpec {
    double mass;     /* kg */
    double length;   /* m, nose to tail */
    double width;    /* m */
    double inertia;  /* kg*m^2 about the centre of mass, precomputed */

    int        thruster_count;          /* 1 .. RF_MAX_THRUSTERS (16) */
    RfThruster thrusters[RF_MAX_THRUSTERS];  /* entries past the count are zeroed */
} RfRocketSpec;
```

```c
typedef struct RfThruster {
    RfVec2 mount;          /* body frame, metres from the centre of mass.
                            * This is what produces torque: a thruster whose line
                            * of action passes through the CoM produces none. */
    double direction;      /* rad, body frame. Direction of the force applied *to
                            * the hull*. A tail engine pushes the hull forward, so
                            * its direction is 0 even though the exhaust goes the
                            * other way. */
    double min_thrust;     /* N. Once lit it cannot go below this. */
    double max_thrust;     /* N */
    double ignition_time;  /* s between being commanded on and producing anything */
    double ramp_up;        /* N/s once burning */
    double ramp_down;      /* N/s */
    double max_gimbal;     /* rad of deflection either side of straight; 0 = fixed */
    double gimbal_rate;    /* rad/s of slew. A nozzle travels; it does not snap. */
} RfThruster;
```

Everything that differs between vehicles is here. **Do not hardcode a layout.** Four airframes ship
today and more can be added as JSON without recompiling anything, including your submission — a
controller that derives its capability from this struct flies airframes that did not exist when it
was written, and one that assumes `classic` scores badly on the other three. The two extremes
shipped today are `norcs`, which has no attitude thrusters at all and can only steer by gimballing a
lit engine, and `lander`, which has real side-thrusting authority and never needs to point its nose
where it is going. Both come out of this struct; nothing announces which is which.

Thruster **names** are deliberately absent from the ABI. Nothing in the physics or the allocator
reads them, and a submission that keyed off one would stop being portable to a layout somebody else
authored.

### `rf_reset`

```c
RF_EXPORT void rf_reset(void);
```

Clear per-episode state: integrators, filters, previous errors, whatever you are carrying between
steps. The spec stays.

### `rf_resolve`

```c
RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out);
```

The job. Called every 10 physics ticks — **100 Hz of simulated time** — and the result is
zero-order held in between, so whatever you write is in force for a full 10 ms of simulated time
regardless of what you do next.

`*out` is **zeroed by the host before the call**, so writing nothing is a valid "all thrusters off"
rather than stack garbage or last step's answer. Neither pointer may be retained after the call
returns. This function must not throw, must not unwind, and **must terminate**.

#### `RfIntent` — what the pilot wants

```c
typedef struct RfIntent {
    int    mode;        /* RF_MODE_ACCELERATION (0) or RF_MODE_VELOCITY (1) */
    RfVec2 vector;      /* meaning depends on mode -- see below. THIS IS THE TRAP. */
    int    has_facing;  /* 0 means "don't care" */
    double facing;      /* rad, world frame */
    int    fire;        /* weapons trigger; wired through, nothing shoots yet */
} RfIntent;
```

`has_facing` is **always 0** in the benchmark. Where to point is your judgement, and the benchmark
scores that judgement rather than your ability to obey an attitude order. On `norcs` that judgement
is most of the job; on `lander` it is barely any of it.

#### `RfObservation` — what is actually happening

```c
typedef struct RfObservation {
    unsigned long long tick;  /* physics ticks since the episode began, 1 kHz */
    double             time;  /* seconds; tick * 0.001 */

    RfVec2 pos;      /* world frame, m */
    RfVec2 vel;      /* world frame, m/s */
    double angle;    /* rad, world frame; 0 points along world +x */
    double ang_vel;  /* rad/s, positive counter-clockwise */

    double max_accel;      /* m/s^2 available along the nose */
    double max_ang_accel;  /* rad/s^2 from thruster placement alone; 0 on a vehicle
                            * that can only steer by gimballing */

    int             thruster_count;
    RfThrusterState actuators[RF_MAX_THRUSTERS];
} RfObservation;

typedef struct RfThrusterState {
    double thrust;          /* N actually being produced RIGHT NOW */
    double gimbal_angle;    /* rad the nozzle has ACTUALLY REACHED */
    double ignition_timer;  /* s remaining before it lights; 0 if not igniting */
    int    lit;             /* nonzero once burning, or committed to burning */
} RfThrusterState;
```

The benchmark runs in the **zero-g world**. There is no gravity, no drag, and no attractor — which
is why the observation carries none. In an orbit, gravity would contribute to every measured
acceleration and every velocity error, and the score would be partly a mark for orbital mechanics
you were never asked to do.

#### `RfControlInput` — your answer

```c
typedef struct RfControlInput {
    double level[RF_MAX_THRUSTERS];   /* [0, 1]  throttle demand */
    double gimbal[RF_MAX_THRUSTERS];  /* [-1, 1] fraction of max_gimbal;
                                       * ignored for a fixed nozzle */
    int    fire;
} RfControlInput;
```

A **demand**, not a result. See `RfThrusterState` for what actually happens to it.

The host clamps `level` to `[0, 1]` and `gimbal` to `[-1, 1]`, and replaces any **non-finite** value
with `0` — not with the range limit. Clamping a NaN keeps it, and clamping an infinity to `1.0`
would turn "this controller computed nonsense" into "this controller commanded hard over", which is
a command you did not mean *and one that scores*. Do not rely on the sanitiser: a clamped command
is a command you did not mean either.

### Units and sign conventions, all of them

| Quantity | Unit |
| --- | --- |
| Position, mount offset, length, width | metres |
| Velocity | m/s |
| Acceleration | m/s² |
| Angle, gimbal deflection | **radians** (the JSON vehicle files use degrees; the ABI does not) |
| Angular velocity | rad/s |
| Angular acceleration | rad/s² |
| Thrust | newtons |
| Ramp rate | N/s |
| Gimbal rate | rad/s |
| Time, ignition delay | seconds |
| Impulse | N·s |

- Positive rotation is **counter-clockwise**.
- Body frame **+x is the nose**. An angle of `0` points along world +x.
- `mount` is in the body frame, metres, relative to the centre of mass.
- `direction` is the direction of the force applied **to the hull**, not the direction of the
  exhaust.

---

## The two things that are easy to get wrong

These are worth their own section because both produce a controller that *looks* like it works and
scores like it does not.

### 1. `RfIntent::vector` changes units with the mode

```c
if (intent->mode == RF_MODE_ACCELERATION) {
    /* A world-frame DIRECTION whose magnitude is <= 1 and is a FRACTION OF THE
     * VEHICLE'S MAXIMUM ACCELERATION. Multiply by obs->max_accel to get m/s^2.
     * The benchmark draws magnitudes uniformly in [0.3, 1.0]. */
} else {
    /* A world-frame TARGET VELOCITY, in ABSOLUTE m/s. Do not scale it by
     * anything. The benchmark draws magnitudes uniformly in [0, 100] m/s. */
}
```

The two are never interleaved inside one run — their commanded quantities have different units and
a tracking error integrated across both would be adding metres to m/s — but **both modes are
evaluated**, and a controller that treats `vector` the same way in each will score catastrophically
in one of them while looking fine in the other. In acceleration mode a magnitude of `1.0` means
"everything you have"; in velocity mode `1.0` means "one metre per second", which is almost
stationary.

### 2. The observation carries *actual* actuator state, not commanded

`RfThrusterState::thrust` is the thrust the vehicle is **producing**. `gimbal_angle` is the angle
the nozzle has **reached**. Neither is what you asked for last step, and the gap between them is the
entire control problem:

- **Ignition delay.** A thruster commanded on produces *nothing* for `ignition_time` — 0.25 s on
  some vehicles, which is 25 control steps of the correction not being there yet.
- **A thrust floor.** Once lit it cannot be throttled below `min_thrust`. It is off, or it is
  somewhere in `[min_thrust, max_thrust]`, and nothing in between at any price. Allocation is
  therefore partly discrete, and **you** must own that rounding rather than let the simulation do it
  silently.
- **Ramp rates.** Thrust moves toward its demand at a finite N/s and decays rather than vanishing.
- **Nozzle slew.** Gimbals travel at `gimbal_rate`. *Reversing* one takes real time — on `norcs`,
  stop to stop is 1.6 s, during which the nozzle is actively turning you the wrong way.

A controller that only knows its own last command is flying blind through its own dead time: it
re-commands a correction that is already on its way and overshoots every time, in both directions.
Reading `actuators[]` is what makes the problem solvable, and it is the single highest-value thing
you can do with this ABI.

Two consequences worth naming, because they are the difference between a controller that scores and
one that oscillates:

- **Attitude needs a deadband.** Below some error the smallest available correction is larger than
  the error itself, and chasing it is a limit cycle wearing a control loop's clothes. Compute that
  limit from the layout.
- **Rate profiles need dead-time compensation.** A gimbal-only vehicle brakes a turn by swinging its
  nozzle all the way across. A profile that assumes braking is instant commits to a rate it cannot
  stop from.

---

## The manifest

A JSON object sent **beside** the artifact, not inside it. Reading a name out of a `.so` would mean
`dlopen`ing it, and `dlopen` runs the library's constructors — a server that had to execute
untrusted code to find out what it was called would have lost the argument before it started.

```json
{
  "name": "my-controller",
  "description": "Dead-time-compensated PD on attitude, discrete allocation on throttle.",
  "author": "someone",
  "language": "c",
  "abi_version": 1,
  "entry": "flybywire.c"
}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `name` | **yes** | Shown on the board. Also slugified into the submission's directory name. |
| `description` | **yes** | Shown on the board. A board of anonymous numbers teaches nobody anything, so an empty one is a 400. |
| `author` | no | Free text. |
| `language` | no | `c`, `cpp`, or `binary` for a prebuilt `.so`. Omit it and it is inferred from the artifact's extension: `.c` → `c`, `.cpp`/`.cc`/`.cxx` → `cpp`, `.so` → `binary`. `c++`, `cxx` and `cc` are accepted as spellings of `cpp`. |
| `abi_version` | no | Defaults to 1. Anything else is a 400. |
| `entry` | no | Documentation only; the server evaluates whatever file you uploaded. |

Every field can also be sent as a plain form field, which **overrides** the manifest. That is how
you submit a source file whose manifest says `cpp` as a prebuilt binary, or fix a typo without
editing the file.

---

## Building the shared object

You may submit **source** (the server compiles it) or a **prebuilt `.so`** (it does not). Source is
better: the operator can read it, two submissions are a text diff, and you get the server's compiler
diagnostics back in your result.

### The exact command the server uses

If you sent `.c` or `.cpp`, this is what runs — reproduce it locally and you will not be surprised:

```sh
# C
cc  -std=c99   -O2 -ffp-contract=off -fPIC -shared -fvisibility=hidden -Wall -Wextra \
    -I <abi-include-dir> source.c   -o artifact.so -lm

# C++
c++ -std=c++20 -O2 -ffp-contract=off -fPIC -shared -fvisibility=hidden -Wall -Wextra \
    -I <abi-include-dir> source.cpp -o artifact.so -lm
```

The full command line is echoed back in your submission's `compile_log`, along with every warning —
`-Wall` on somebody else's code is the cheapest review there is, and a warning you never see is one
you will never fix.

`-ffp-contract=off` is not decoration. Fusing `a*b+c` into an FMA changes results between
optimisation levels, and a submission whose score depends on how the server felt like compiling it
is not a submission that can be ranked. Use it in your local builds too.

Note there is **no `-I src`**. A source submission can include the ABI header and the standard
library and nothing else from this repository. (The one exception is
[`examples/submissions/reference_cpp`](examples/submissions/reference_cpp), which deliberately wraps
the project's own `RocketFlyByWire`; it is built by this repo's CMake and submitted as a
`binary`. It is a baseline, not a template.)

### From C

```sh
cc -std=c99 -O2 -ffp-contract=off -fPIC -shared -fvisibility=hidden \
   -I include flybywire.c -o flybywire.so -lm
```

Worked example: [`examples/submissions/simple_c/flybywire.c`](examples/submissions/simple_c/flybywire.c)
— about 200 lines, includes nothing but the ABI header and `math.h`, flies all four airframes, and
is deliberately easy to beat.

### From C++

Declare the entry points `extern "C"`. Forgetting is by far the most common submission failure, and
the host says so by name when it happens.

```cpp
#include "rocketfight/flybywire_abi.h"
#include <vector>

extern "C" {
RF_EXPORT int  rf_abi_version(void) { return RF_ABI_VERSION; }
RF_EXPORT int  rf_init(const RfRocketSpec* spec);
RF_EXPORT void rf_reset(void);
RF_EXPORT void rf_resolve(const RfIntent*, const RfObservation*, RfControlInput*);
RF_EXPORT void rf_shutdown(void);
}
```

```sh
c++ -std=c++20 -O2 -ffp-contract=off -fPIC -shared -fvisibility=hidden \
    -I include flybywire.cpp -o flybywire.so
```

**An exception must not escape any of the five functions.** Unwinding through a C frame is undefined
behaviour and the host cannot catch what it cannot see. Catch at the boundary:

```cpp
RF_EXPORT void rf_resolve(const RfIntent* in, const RfObservation* obs, RfControlInput* out) try {
    ...
} catch (...) {
    /* `*out` was zeroed by the host, so bailing out here is "everything off"
     * for one control step rather than undefined behaviour. */
}
```

If one does escape and the process survives long enough to unwind that far, the runner reports
`submission threw: <what>` and the episode is a recorded failure. If it does not unwind cleanly, the
process dies and you get `crashed: killed by signal 6 (Aborted)` instead. Both are results; neither
is a good one.

### From Rust

Build a `cdylib` with `#[no_mangle] pub extern "C"`. Submit the resulting `.so` as
`language=binary`; the server compiles C and C++ only.

```rust
// flybywire.rs  --  no external crates, no build system required
#![allow(non_camel_case_types)]

pub const RF_MAX_THRUSTERS: usize = 16;
pub const RF_ABI_VERSION: i32 = 1;
pub const RF_MODE_ACCELERATION: i32 = 0;
pub const RF_MODE_VELOCITY: i32 = 1;

#[repr(C)] #[derive(Copy, Clone, Default)]
pub struct RfVec2 { pub x: f64, pub y: f64 }

#[repr(C)] #[derive(Copy, Clone, Default)]
pub struct RfThruster {
    pub mount: RfVec2, pub direction: f64,
    pub min_thrust: f64, pub max_thrust: f64, pub ignition_time: f64,
    pub ramp_up: f64, pub ramp_down: f64,
    pub max_gimbal: f64, pub gimbal_rate: f64,
}

#[repr(C)]
pub struct RfRocketSpec {
    pub mass: f64, pub length: f64, pub width: f64, pub inertia: f64,
    pub thruster_count: i32,
    pub thrusters: [RfThruster; RF_MAX_THRUSTERS],
}

#[repr(C)] #[derive(Copy, Clone, Default)]
pub struct RfThrusterState {
    pub thrust: f64, pub gimbal_angle: f64, pub ignition_timer: f64, pub lit: i32,
}

#[repr(C)]
pub struct RfObservation {
    pub tick: u64, pub time: f64,
    pub pos: RfVec2, pub vel: RfVec2, pub angle: f64, pub ang_vel: f64,
    pub max_accel: f64, pub max_ang_accel: f64,
    pub thruster_count: i32,
    pub actuators: [RfThrusterState; RF_MAX_THRUSTERS],
}

#[repr(C)]
pub struct RfIntent {
    pub mode: i32, pub vector: RfVec2,
    pub has_facing: i32, pub facing: f64, pub fire: i32,
}

#[repr(C)]
pub struct RfControlInput {
    pub level: [f64; RF_MAX_THRUSTERS],
    pub gimbal: [f64; RF_MAX_THRUSTERS],
    pub fire: i32,
}

static mut COUNT: i32 = 0;

#[no_mangle] pub extern "C" fn rf_abi_version() -> i32 { RF_ABI_VERSION }

#[no_mangle] pub extern "C" fn rf_init(spec: *const RfRocketSpec) -> i32 {
    // A panic must not cross this boundary; catch_unwind, or write code that
    // cannot panic. Indexing and unwrap() both can.
    unsafe { COUNT = (*spec).thruster_count; }
    0
}

#[no_mangle] pub extern "C" fn rf_reset() {}

#[no_mangle] pub extern "C" fn rf_resolve(
    _intent: *const RfIntent, _obs: *const RfObservation, out: *mut RfControlInput,
) {
    unsafe { for i in 0..COUNT as usize { (*out).level[i] = 0.0; } }
}

#[no_mangle] pub extern "C" fn rf_shutdown() {}
```

```sh
rustc --edition 2021 --crate-type=cdylib -C opt-level=2 -C panic=abort \
      -o flybywire.so flybywire.rs
```

`-C panic=abort` because a Rust panic unwinding through the host's C frame is undefined behaviour,
exactly as a C++ exception is. Aborting turns "undefined" into "this episode crashed", which is a
result the board can display. Better still, use `std::panic::catch_unwind` at each entry point and
return a safe answer.

> The Rust path is *not* built or tested by this repository's CI — there is no Rust toolchain on the
> reference host. The struct definitions above are transcribed from the header and the layout is
> `#[repr(C)]`, so they are correct by construction, but you are the one running the compiler.
> `rf_abi_version()` returning 1 and a run producing a plausible score is your evidence that the
> layout matched. Verify locally with `rocketfight_runner` before you submit; see
> [Testing locally](#testing-locally-before-you-submit).

---

## Submitting

`multipart/form-data` to `POST /api/submissions`.

```sh
curl -X POST http://HOST:PORT/api/submissions \
     -F artifact=@flybywire.c \
     -F manifest=@manifest.json
```

or without a manifest file:

```sh
curl -X POST http://HOST:PORT/api/submissions \
     -F artifact=@flybywire.so \
     -F name="my-controller" \
     -F description="Dead-time-compensated PD, discrete allocation." \
     -F language=binary
```

You get `202 Accepted` immediately — evaluation is queued, not synchronous:

```json
{
  "id": "0004-my-controller",
  "status": "queued",
  "poll": "/api/submissions/0004-my-controller"
}
```

### Polling

```sh
curl -s http://HOST:PORT/api/submissions/0004-my-controller
```

`status` walks `queued` → `compiling` → `evaluating` → one of:

| Status | Ranked? | Meaning |
| --- | --- | --- |
| `ok` | yes | Every configuration that ran, ran; it reproduced. |
| `nondeterministic` | **no** | Two runs of the same configuration disagreed. Numbers are published so you can see them; the entry is not ranked. |
| `crashed` | no | No configuration completed. |
| `rejected` | no | Would not compile, or could not be stored. |

A submission can be `ok` with some runs failed — the failures are charged the failure penalty and
`message` says how many.

The full record carries per-run detail: every `(rocket, mode, seed)` triple with its status, score,
`state_hash`, wall time and all four metric groups, plus `per_rocket` and `per_mode` means, plus the
compiler's output.

### The board

| Endpoint | What |
| --- | --- |
| `GET /` | The leaderboard as an HTML page, refreshing every 5 s. |
| `GET /api/leaderboard` | The same thing as JSON, including the live weights. |
| `GET /api/submissions/<id>` | One submission in full. |
| `POST /api/submissions` | Submit. |

```jsonc
{
  "generated_at": "2026-07-26T20:05:41Z",
  "benchmark": "flybywire-tracking",
  "abi_version": 1,
  "rockets": ["classic", "interceptor", "lander", "norcs"],
  "modes": ["acceleration", "velocity"],
  "weights": { /* the live ranking policy, published so you can recompute it */ },
  "entries": [
    {
      "id": "0001-reference-cpp",
      "name": "reference-cpp",
      "description": "The project's built-in RocketFlyByWire wrapped in the C ABI.",
      "author": "rocketfight",
      "language": "binary",
      "submitted_at": "2026-07-26T20:05:31Z",
      "evaluated_at": "2026-07-26T20:05:32Z",
      "status": "ok",
      "message": "",
      "aggregate": 37.87665869192271,
      "ranked": true,
      "deterministic": true,
      "runs_ok": 24,
      "runs_failed": 0,
      "per_rocket": { "classic": 31.63, "interceptor": 35.54,
                      "lander": 41.83, "norcs": 42.49 },
      "per_mode":   { "acceleration": 17.56, "velocity": 58.18 }
    }
  ]
}
```

Ranked entries come first, ascending — **lower is better**. Everything that could not be ranked
follows, in submission order, visible rather than dropped: hiding failures would make the board look
better than the submissions are.

---

## What is measured, and how it is scored

### The task

A scripted pilot — `BenchmarkIntentSource` — occupies exactly the slot a human gamepad occupies. It
emits a step sequence and holds each value for an interval **uniform in [1, 5] s** before
resampling. The sequence depends on **the seed alone**: not on the vehicle, not on the rocket's
state, not on what you did with the last command. Two submissions, and two vehicles, face a
byte-identical task.

- **Acceleration mode.** Direction uniform on the circle, magnitude uniform in **[0.3, 1.0]** of the
  vehicle's maximum.
- **Velocity mode.** Direction uniform on the circle, magnitude uniform in **[0, 100] m/s**,
  absolute.
- `has_facing` is always 0.

### The matrix

**Every rocket in `rockets/`, both modes, every seed.** With the four shipped airframes and the
three default seeds that is:

```
4 rockets x 2 modes x 3 seeds = 24 runs of 60 simulated seconds each
                              + 8 determinism repeats (first seed of each rocket/mode pair)
```

Ranking a controller on the vehicle it was tuned against is ranking the tuning, so there is no way
to opt out of an airframe. Several seeds because one seed is a single realisation of a random
sequence and a controller can be lucky once.

Adding a fifth rocket means adding a JSON file — no recompilation, and your submission is evaluated
against it without being resubmitted.

### The four metrics

| Metric | Units | Why |
| --- | --- | --- |
| **Tracking error** | ∫\|commanded − actual\| dt | The task itself. Reported as an integral *and* a mean, since the integral grows with run length. |
| **Settling time** | s, mean and worst | Reaching the right answer eventually is not the same as reaching it. Measured per step change, from the resample to the last moment the error was outside tolerance. |
| **Impulse used** | N·s | Two controllers that track equally well are not equally good if one burns twice the propellant. |
| **Attitude wander** | rad | Integrated \|angular velocity\|, so it is total angle *swept* rather than net rotation — spinning one way and back has done the work twice. Rotation nobody asked for is wasted authority. |

"Settled" means inside **10% of `max_accel`** in acceleration mode, or **5 m/s** in velocity mode.
An unsettled step is charged the whole hold interval rather than dropped — dropping it would let a
controller that *never* settles outscore one that settles slowly, by contributing no samples at all.
`settled_fraction` is published next to it so the two stay separable.

Actual acceleration is computed **from the vehicle**, by summing the wrench from `ThrusterState`,
not by differencing positions. Differencing a trajectory would measure the integrator.

### The score

Per run:

```
score = tracking * tracking_error_mean
      + settling * settling_time_mean_s
      + impulse  * mean_thrust_n
      + wander   * mean_ang_rate_rad_s
```

with the default weights:

```json
{ "tracking": 1.0, "settling": 1.0, "impulse": 1e-05, "wander": 1.0 }
```

Per submission, the `aggregate` is the weighted mean of that across all 24 runs, where each run's
weight is `rocket_weight * mode_weight` (both default to **1.0**, and a rocket or mode absent from
the weight map weighs 1.0 rather than 0.0 — so a new airframe joins the aggregate on its own instead
of silently not counting). A failed run is charged the **failure penalty**, `1000.0`, which is by
construction worse than any plausible real score. Crashing on the hard vehicle is not a strategy.

### The weights are data and will be retuned

They live in the operator's `weights.json`, they are published on every leaderboard response, and
they are explicitly a guess made before any real submission existed. **Do not overfit to them.**

Because every per-run component is archived, re-weighting the board is a re-read of stored results
rather than a re-run of anything — an existing submission's numbers do not change when the policy
does, only its position. The same file also owns `seeds`, `seconds`, `failure_penalty` and
`verify_determinism`.

---

## The limits

Real numbers, read out of [`src/server/Sandbox.cpp`](src/server/Sandbox.cpp) and
[`src/server/Compile.cpp`](src/server/Compile.cpp). Everything here is applied in the child between
`fork` and `exec`, so it is in force before a single instruction of your code runs.

### Per evaluation job — one `(rocket, mode, seed)` triple

| Limit | Value | Mechanism |
| --- | --- | --- |
| **Wall clock** | **10 s** | Deadline in the parent. `SIGKILL` to the whole process group. Not `SIGTERM`. |
| CPU time | 5 s soft, 6 s hard | `RLIMIT_CPU`. Soft raises `SIGXCPU`; the hard limit is the `SIGKILL` behind it. |
| Address space | 1 GiB | `RLIMIT_AS` |
| File size | 16 MiB | `RLIMIT_FSIZE` |
| Open files | 64 | `RLIMIT_NOFILE` |
| Processes / threads | 64 | `RLIMIT_NPROC` (caps threads too, on Linux) |
| Core dump size | 0 | `RLIMIT_CORE`. A crash is a normal, frequent result; each core would be a multi-hundred-megabyte image of the runner. |
| stdout + stderr | 4 MiB | Enforced by the parent. Pipes are not covered by `RLIMIT_FSIZE`, so a submission printing in a loop would otherwise fill the *server's* memory. Exceeding it kills the job. |

For scale: a 60-second run of the reference controller takes about **20 ms**. The 10-second deadline
is 500× that, and 1.6 ms of budget per `rf_resolve` call. A submission that hits it was not merely
slow.

**The two limits catch different things, which is why both exist.** A submission that *busy-spins*
burns CPU and dies to `RLIMIT_CPU` first, at about 5 s. A submission that *blocks* — one `sleep`, one
read from a pipe nobody writes to — consumes no CPU and no memory and sails past every rlimit there
is; only the wall-clock deadline ends it, at 10.00 s. Measured on the reference host, across 24 jobs
each: the spinner died at 5.06–5.10 s wall, the sleeper at 10.0004–10.0025 s.

### Per compile — only if you sent source

| Limit | Value |
| --- | --- |
| Wall clock | 60 s |
| CPU time | 45 s |
| Address space | 4 GiB (template expansion is hungry) |
| File size | 64 MiB |
| Open files | 256 |
| Processes | 64 (the driver forks `cc1` and the linker) |
| Compiler output captured | 1 MiB |

### Environment and filesystem

- The child's environment is **replaced**, not extended. It sees exactly
  `PATH=/usr/bin:/bin`, `ROCKETFIGHT_ROCKETS`, `HOME`, `LC_ALL=C` — and nothing the server was
  started with. A server's own environment routinely holds tokens and keys that have nothing to do
  with flying a rocket.
- The working directory is a scratch directory. If you write a file, it goes somewhere disposable.
- `stdin` is `/dev/null`, so a submission that reads it sees EOF rather than blocking forever.
- `PR_SET_NO_NEW_PRIVS` is set, so no setuid binary the child execs can gain privileges.
- The child gets its own session (`setsid`), so the deadline is enforced against the whole process
  group. A submission that forks and then exits does not leave orphans running.
- **Network access is removed on a best-effort basis** via an unprivileged user namespace plus a
  network namespace. Where the kernel or container does not permit that, it is skipped with a
  logged warning rather than failing the run. The server prints which on startup. Do not rely on
  either state.

---

## Determinism

**Determinism is part of the contract.**

The host re-runs the first seed of every `(rocket, mode)` pair **in a second, separate process** and
compares a 64-bit hash of the resulting world state. Two processes rather than two passes in one
address space, deliberately: a submission keyed off an address, a pid or an uninitialised byte
reproduces perfectly inside one process and is only caught across two.

A submission that reads any of these will be flagged:

- a clock of any kind
- `rand()`, `/dev/urandom`, `getrandom()`, hardware RNG
- a thread id, a process id, or a pointer value
- an uninitialised variable
- environment or filesystem state
- iteration order of a hash container
- anything concurrent

Seeded pseudo-randomness is **fine**. Seed it from something inside the ABI — the spec, the tick,
the observation — not from the environment.

### What happens to a submission that fails it

- `status` becomes `nondeterministic`, and `ranked` is **false**.
- Its numbers are still published, in full, so you can see what it scored and fix it.
- `determinism_note` names the configuration and both hashes:
  `classic/acceleration: 0x0b12f918d453f344 then 0xdc1cd4a1072e8bdb`.
- It never affects anyone else's position.

This is deliberately harsh. One nondeterministic entry makes every comparison on the board
meaningless — including its own, since the score published is one realisation of a number that will
not happen again.

The host's own determinism guarantee is **same machine, same binary, bit-exact**. Cross-machine
exactness is explicitly not promised. `-ffp-contract=off` and the absence of `-ffast-math` are part
of holding that up, on your side as well as the server's.

---

## The sandbox is hardening, not a security boundary

Read this before deploying the server anywhere that matters.

`setrlimit` plus a wall-clock kill timer is **hardening against buggy and casually hostile code** —
an infinite loop, a runaway allocation, a stray `fopen`, a fork bomb, a wild pointer, a submission
that tries to phone home. It is **not** a security boundary against a determined attacker.

A determined attacker defeats it, because **arbitrary native code execution is exactly what a `.so`
submission is**. The only real answers to that are a container, a VM, or a seccomp policy far
stricter than anything here.

What it explicitly does **not** do:

- **No filesystem confinement.** The child can read anything the invoking user can read. There is no
  chroot, no mount namespace, no bind-mounted view.
- **No syscall filtering.** No seccomp-bpf.
- **No defence against a submission that attacks the kernel.**
- **Network isolation is best-effort and may be silently absent** where unprivileged user namespaces
  are not permitted. The server logs which on startup.

What it *does* buy, honestly stated: a deadline the child cannot influence, rlimits in force before
the child's first instruction, a scrubbed environment, a disposable working directory, and a process
boundary that means a crash or a hang costs one job rather than the service.

The server **refuses to run as root** unless passed `--allow-root`. Every limit it applies is one
root can lift, and a submission that escaped would escape as root.

**If you are running this against untrusted submissions from the open internet, put it in a
container or a VM.** The process boundary is what makes it survivable, not what makes it safe.

---

## A complete worked example

From nothing to a ranked submission.

### 1. Write it

`flybywire.c`:

```c
/* Point the nose at the commanded direction, then open the forward thrusters
 * when it is roughly there. Not good -- it ignores ignition dead time entirely
 * -- but it flies every airframe and it is a complete, valid submission. */

#include "rocketfight/flybywire_abi.h"
#include <math.h>

#define PI 3.14159265358979323846

/* Solved once in rf_init, because rf_resolve runs 6000 times a run. */
static struct {
    int    count;
    double lever[RF_MAX_THRUSTERS];       /* torque per newton, signed */
    double forwardness[RF_MAX_THRUSTERS]; /* push along the nose, per newton */
    double gimbal_arm[RF_MAX_THRUSTERS];  /* torque per newton per rad of nozzle */
    double max_gimbal[RF_MAX_THRUSTERS];
    double max_accel;
    double inertia;
} g;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Shortest signed angle from `from` to `to`. Getting this wrong is how a
 * controller decides to turn 359 degrees the long way round. */
static double wrap(double a) {
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

RF_EXPORT int rf_abi_version(void) { return RF_ABI_VERSION; }

RF_EXPORT int rf_init(const RfRocketSpec* spec) {
    int    i;
    double forward_thrust = 0.0;

    g.count   = spec->thruster_count;
    g.inertia = spec->inertia;

    for (i = 0; i < g.count; ++i) {
        const RfThruster* t  = &spec->thrusters[i];
        const double      dx = cos(t->direction);
        const double      dy = sin(t->direction);

        /* Torque = r x F, in 2D a scalar. Zero for a thruster whose line of
         * action passes through the centre of mass -- which is exactly how a
         * main engine differs from an attitude puffer, derived rather than
         * assumed. */
        g.lever[i]       = t->mount.x * dy - t->mount.y * dx;
        g.forwardness[i] = dx;
        /* Deflecting the nozzle rotates the force; to first order the extra
         * torque per radian is the moment arm along the hull. */
        g.gimbal_arm[i]  = t->mount.x * dx + t->mount.y * dy;
        g.max_gimbal[i]  = t->max_gimbal;

        if (dx > 0.5) forward_thrust += t->max_thrust;
    }

    g.max_accel = spec->mass > 0.0 ? forward_thrust / spec->mass : 0.0;
    /* Decline a vehicle we cannot push at all rather than score zero on it. */
    return g.max_accel > 0.0 ? 0 : 1;
}

RF_EXPORT void rf_reset(void) {}

RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    int    i;
    double want_x, want_y, mag, want_heading, err, rate_cmd, torque_cmd, throttle;

    /* --- 1. Turn the intent into a world-frame acceleration in m/s^2. ------
     * THIS is the part that differs between the two modes, and getting it
     * wrong is the single most common way to score badly. */
    if (intent->mode == RF_MODE_ACCELERATION) {
        /* Already a fraction of max. Scale it up. */
        want_x = intent->vector.x * obs->max_accel;
        want_y = intent->vector.y * obs->max_accel;
    } else {
        /* An ABSOLUTE target velocity in m/s. Proportional closing on the
         * velocity error, saturated at what the vehicle actually has. */
        const double ex = intent->vector.x - obs->vel.x;
        const double ey = intent->vector.y - obs->vel.y;
        want_x = clampd(ex * 2.0, -obs->max_accel, obs->max_accel);
        want_y = clampd(ey * 2.0, -obs->max_accel, obs->max_accel);
    }

    mag = sqrt(want_x * want_x + want_y * want_y);
    if (mag < 1e-9) return;  /* *out is already zeroed: everything off */

    /* --- 2. Point the nose at it. ---------------------------------------- */
    want_heading = atan2(want_y, want_x);
    err          = wrap(want_heading - obs->angle);

    /* A rate profile: ask for a turn rate proportional to the error, then a
     * torque proportional to the rate error. Deliberately naive -- it takes no
     * account of ignition delay or nozzle slew, which is most of what a good
     * submission would add here. */
    rate_cmd   = clampd(err * 2.0, -1.5, 1.5);
    torque_cmd = (rate_cmd - obs->ang_vel) * g.inertia * 2.0;

    /* --- 3. Throttle only once roughly aligned. --------------------------- */
    throttle = fabs(err) < 0.35 ? clampd(mag / obs->max_accel, 0.0, 1.0) : 0.0;

    for (i = 0; i < g.count; ++i) {
        if (g.forwardness[i] > 0.5) {
            out->level[i] = throttle;
            if (g.max_gimbal[i] > 0.0 && fabs(g.gimbal_arm[i]) > 1e-9) {
                /* Steer with the nozzle. Note this only works while lit --
                 * which is why the throttle gate above is a real cost on a
                 * vehicle with no attitude thrusters. */
                const double want = torque_cmd / (g.gimbal_arm[i] * 1000.0);
                out->gimbal[i] = clampd(want, -1.0, 1.0);
            }
        } else if (g.lever[i] * torque_cmd > 0.0) {
            /* An attitude thruster that torques the way we want. Vehicles
             * without any get nothing here and must steer by gimbal alone. */
            out->level[i] = clampd(fabs(torque_cmd) / (fabs(g.lever[i]) * 4000.0),
                                   0.0, 1.0);
        }
    }
}

RF_EXPORT void rf_shutdown(void) {}
```

### 2. Build it

```sh
cc -std=c99 -O2 -ffp-contract=off -fPIC -shared -fvisibility=hidden \
   -Wall -Wextra -I include flybywire.c -o flybywire.so -lm
```

Check the five symbols are actually exported — `-fvisibility=hidden` plus a forgotten `RF_EXPORT` is
a silent way to fail:

```sh
nm -D --defined-only flybywire.so | grep rf_
# 00000000000003b0 T rf_abi_version
# 00000000000003c0 T rf_init
# 0000000000000510 T rf_reset
# 0000000000000520 T rf_resolve
# 0000000000000830 T rf_shutdown
```

Five `T` entries and nothing else. A `t` instead of a `T`, or a missing line, means `RF_EXPORT` was
left off and the host will refuse the library by name.

### 3. Try it locally

```sh
./build/rocketfight_runner --rocket=classic --mode=accel --seconds=60 \
                           --library=$PWD/flybywire.so --pretty
```

### 4. Write the manifest

`manifest.json`:

```json
{
  "name": "heading-and-gate",
  "description": "Points the nose at the commanded acceleration, then gates the throttle on alignment. Ignores ignition dead time.",
  "author": "you",
  "language": "c",
  "abi_version": 1
}
```

### 5. Submit

```sh
curl -X POST http://HOST:PORT/api/submissions \
     -F artifact=@flybywire.c \
     -F manifest=@manifest.json
```

```json
{ "id": "0004-heading-and-gate", "status": "queued",
  "poll": "/api/submissions/0004-heading-and-gate" }
```

### 6. Poll

```sh
curl -s http://HOST:PORT/api/submissions/0004-heading-and-gate | head -40
```

Under a second later, if it worked:

```json
{
  "id": "0004-heading-and-gate",
  "name": "heading-and-gate",
  "status": "ok",
  "aggregate": 37.1001026864482,
  "ranked": true,
  "deterministic": true,
  "runs_ok": 24,
  "runs_failed": 0,
  "per_rocket": { "classic": 35.26, "interceptor": 40.87,
                  "lander": 30.65, "norcs": 41.62 },
  "per_mode":   { "acceleration": 20.95, "velocity": 53.25 }
}
```

Those are the real numbers this file produces, measured. Two things in them are worth reading.

**Read `per_rocket` before anything else.** `norcs` at 41.6 against `lander` at 30.7 is the
controller telling you exactly what is wrong with it: `norcs` has no attitude thrusters, so its only
way to turn is to deflect a nozzle on a *lit* engine — and this controller gates the throttle on
already being aligned. It shuts off the only thing that could have turned it, then waits. Add
`fabs(err) < 0.35` to the list of decisions you should not have made.

**Read `per_mode` too.** Velocity mode costs 53.2 against acceleration's 21.0, and the gap is not
this controller being bad at velocity — it is the units. Velocity-mode errors are in m/s over a
0–100 m/s band, acceleration-mode errors are in m/s² over a band the vehicle can actually reach. The
two columns are not comparable to each other; they are comparable to *other submissions'* columns.

This example scores about the same as the built-in reference (37.9). That is not a claim that a
throttle gate is as good as dead-time compensation — it is the default weights being a guess, and
`impulse` at `1e-05` in particular barely penalising a controller that keeps its engines off half
the time. It is a fair illustration of why [the weights are data and will be
retuned](#the-weights-are-data-and-will-be-retuned), and of why overfitting to them is a bad trade.

---

## Testing locally before you submit

`rocketfight_runner` is the exact binary the server forks. Same code path, same results.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j

# one configuration
./build/rocketfight_runner --rocket=classic --mode=accel --seconds=60 \
                           --seed=7 --library=$PWD/flybywire.so --pretty

# the built-in fly-by-wire, same loop, for comparison: omit --library
./build/rocketfight_runner --rocket=classic --mode=accel --seconds=60 --seed=7 --pretty
```

| Flag | Default | Effect |
| --- | --- | --- |
| `--rocket=NAME` | required | `classic`, `norcs`, `lander`, `interceptor`, … |
| `--mode=accel\|velocity` | `accel` | which experiment |
| `--seconds=N` | 60 | simulated seconds; refused above 300 (see below) |
| `--seed=N` | fixed default | accepts `0x…` |
| `--library=PATH` | built-in | your `.so`. **Must contain a `/`** — a bare filename would send `dlopen` hunting the loader search path. |
| `--pretty` | off | indented JSON |

The path must be absolute or contain a `/`; `--library=flybywire.so` is refused on purpose.

### Run length is capped at 300 simulated seconds

Scored runs are 60 s, and if you are experimenting locally you can go up to 300 s. Beyond that the
flag is refused, and the reason is worth knowing because it tells you something about the task.

The benchmark deliberately never checks whether the vehicle has left the 1000 km world bound. Every
run integrates over exactly the same number of ticks no matter how far it travelled — it has to,
because acceleration mode is a random walk in velocity and a stronger airframe covers more ground
under the identical command sequence. Cutting a run short for drifting would make its
time-integrated metrics incomparable with everyone else's.

That is safe because position feeds nothing: the benchmark world has no gravity, and no metric or
controller reads position. But drift is close to ballistic rather than a random walk, because **no
rocket in the catalogue has a retro thruster** — a command with a rearward component produces
nothing rearward, leaving a standing bias along the nose. Measured, vehicles begin crossing the
bound from about 450 s. At 60 s the margin is roughly 17x.

If you are wondering whether to exploit that bias: it applies identically to every submission on
every airframe, so there is nothing to win from it. It is a property of the vehicles, not a hole in
the scoring.

**Check your own determinism before the server does**, which is two commands:

```sh
for i in 1 2; do
  ./build/rocketfight_runner --rocket=classic --mode=accel --seconds=60 \
      --library=$PWD/flybywire.so | grep -o '"state_hash":"[^"]*"'
done
# the two lines must be identical
```

Sweep everything the server will:

```sh
for r in classic norcs lander interceptor; do
  for m in accel velocity; do
    printf '%-12s %-9s ' "$r" "$m"
    ./build/rocketfight_runner --rocket=$r --mode=$m --seconds=60 \
        --library=$PWD/flybywire.so | grep -o '"score":[0-9.]*'
  done
done
```

---

## Failure modes and what they look like

Every one of these is a **recorded result with a reason**, per `(rocket, mode, seed)`. None of them
takes the server down, and none of them stops the other configurations from running — so a
submission that crashes on one vehicle still gets a complete row telling you which.

| What you did | What the board says |
| --- | --- |
| Source that will not compile | `rejected`, `message: compilation failed`, full diagnostics in `compile_log` |
| A `.so` that is not an ELF object | `rejected: dlopen failed: …file too short` |
| Forgot `extern "C"` on a C++ submission | `rejected: missing required symbol 'rf_resolve' (C++ submissions must declare it extern "C")` |
| `rf_abi_version` returns the wrong number | `rejected: ABI version mismatch: library reports 1001, host expects 1` |
| `rf_init` returns nonzero | `rejected: rf_init declined the vehicle 'classic' (returned 7)` |
| Dereferenced a null pointer | `crashed: killed by signal 11 (Segmentation fault)` |
| `rf_resolve` busy-loops | `crashed: killed by signal 24 (CPU time limit exceeded)`, at ~5 s |
| `rf_resolve` blocks forever | `timeout: timed out after 10.0011s and was SIGKILLed` |
| Printed without end | `timeout: SIGKILLed: output exceeded 4194304 bytes` |
| Allocated without end | `crashed`, or a refused allocation — `RLIMIT_AS` is 1 GiB |
| An exception escaped | `rejected: submission threw: <what>`, or `crashed: killed by signal 6 (Aborted)` |
| Returned NaN or ±inf | **Nothing.** Silently replaced with 0 before the physics sees it. You will score badly and no message will tell you why — check your own arithmetic. |
| Two runs disagreed | `nondeterministic`, published but never ranked |

A hostile example that demonstrates the last few is shipped:
[`examples/submissions/hostile_spin/`](examples/submissions/hostile_spin/) — its `rf_resolve` never
returns. It exists to be killed, and the assertions that it *is* killed live in
[`tests/test_submission.cpp`](tests/test_submission.cpp).

---

## Files

| Path | What |
| --- | --- |
| [`include/rocketfight/flybywire_abi.h`](include/rocketfight/flybywire_abi.h) | **The contract.** The only file you need. |
| [`examples/submissions/simple_c/`](examples/submissions/simple_c/) | A complete C99 submission, ~200 lines, deliberately beatable. |
| [`examples/submissions/reference_cpp/`](examples/submissions/reference_cpp/) | The built-in fly-by-wire wrapped in the ABI. The baseline. Built by this repo's CMake — it reaches into `src/`, so it is a `binary` submission and not a template. |
| [`examples/submissions/hostile_spin/`](examples/submissions/hostile_spin/) | A submission that hangs. Ships so the supervision can be demonstrated rather than asserted. |
| [`rockets/`](rockets/) | The airframes you are evaluated against. Read them; they are the problem statement. |
