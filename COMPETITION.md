# RocketFight fly-by-wire competition — everything you need

You write the **fly-by-wire**: the layer that turns a pilot's *intent* ("accelerate that way",
"be moving at that velocity") into per-thruster commands. Nothing else. The pilot, the physics and
the vehicles are fixed; only your controller changes.

Ship a shared object exporting five C functions. Any language with a C ABI works.

Full reference: [`SUBMISSIONS.md`](SUBMISSIONS.md). This page is the whole contract in short form.

---

## Quickstart

```sh
cc -O2 -fPIC -shared -I include examples/submissions/simple_c/flybywire.c -o mine.so

curl -X POST http://HOST:PORT/api/submissions \
     -F artifact=@mine.so -F name="my-controller" \
     -F description="What it does." -F language=binary

curl -s http://HOST:PORT/api/leaderboard
```

Send `flybywire.c` / `.cpp` instead of `.so` and the server compiles it for you (`language: c` or
`cpp`). Copy [`examples/submissions/simple_c`](examples/submissions/simple_c) as your template.
`reference_cpp` is the baseline to beat, **not** a template — it reaches into `src/`, which the
submission compiler deliberately cannot see.

---

## The ABI

`#include "rocketfight/flybywire_abi.h"`. From C++, everything must be `extern "C"`.

```c
int  rf_abi_version(void);              /* return RF_ABI_VERSION */
int  rf_init(const RfRocketSpec* spec); /* once per episode; 0 = ok, nonzero = refuse */
void rf_reset(void);                    /* clear accumulated state */
void rf_resolve(const RfIntent*, const RfObservation*, RfControlInput* out);  /* 100 Hz */
void rf_shutdown(void);
```

`rf_resolve` is called at **100 Hz** while physics runs at 1000 Hz; your output is held for the ten
ticks in between. Never block, never allocate if you can avoid it, never throw across the boundary.

### What you are given

`RfRocketSpec` (constant for the episode) — `mass`, `length`, `width`, `inertia`,
`thruster_count`, and per thruster: `mount`, `direction`, `min_thrust`, `max_thrust`,
`ignition_time`, `ramp_up`, `ramp_down`, `max_gimbal`, `gimbal_rate`.

`RfObservation` (every call) — `tick`, `time`, `pos`, `vel`, `angle`, `ang_vel`, `max_accel`,
`max_ang_accel`, and `actuators[]`: each thruster's **actual** `thrust`, `gimbal_angle`,
`ignition_timer`, `lit`.

`RfIntent` — `mode`, `vector`, `has_facing`, `facing`, `fire`.

### What you write

`RfControlInput` — `level[i]` in `[0,1]` (throttle demand per thruster), `gimbal[i]` in `[-1,1]`
(fraction of that thruster's `max_gimbal`), `fire`.

Units are SI throughout: metres, seconds, radians, kilograms, newtons. World space is **y-up**;
positive angles and `ang_vel` are counter-clockwise; `angle = 0` points along world +x.

### Two things that will catch you out

1. **`RfIntent::vector` means different things in the two modes.** In `RF_MODE_ACCELERATION` it is
   a direction with magnitude ≤ 1, as a **fraction of `max_accel`**. In `RF_MODE_VELOCITY` it is an
   **absolute world-frame target velocity in m/s**. Treating one as the other is the single most
   common way to produce a controller that looks plausible and scores terribly.
2. **`actuators[]` is what the thrusters are *doing*, not what you asked for.** Ignore it and you
   will re-command corrections that are already on their way, and oscillate. It is the only way to
   see an engine that is still inside its ignition delay.

---

## The physics you are fighting

Thrusters are not instantaneous knobs:

- **Ignition delay** — commanded on, produces nothing for `ignition_time`.
- **Thrust floor** — once lit it cannot go below `min_thrust`. Off, or in `[min, max]`. There is no
  small correction available at any price; this is why fine attitude control needs a deadband.
- **Ramp rates** — thrust moves at `ramp_up` / `ramp_down` N/s and decays rather than vanishing.
- **Nozzle slew** — gimbals travel at `gimbal_rate`. *Reversing* a nozzle takes real time.

A thruster mounted off the centreline produces torque; nothing else does. Attitude control is
geometry, not a separate system. Firing diagonally opposite thrusters gives a pure couple (forces
cancel, torques add); firing two on the same side translates without turning.

---

## The vehicles — you are scored on all four

| rocket | character |
| --- | --- |
| `classic` | Gimballed main + two opposed attitude pairs. The reference case. |
| `norcs` | **No attitude thrusters.** Can only steer by deflecting a lit engine, so it must burn to turn — and its nozzle is slow. The hardest, and currently the highest-scoring, vehicle. |
| `lander` | Real side-thrusting authority. Does **not** need to point its nose where it is going. |
| `interceptor` | Twin mains off the centreline, so differential throttle is a second torque source. High floors, slow to light. |

Read the layout from `RfRocketSpec` rather than special-casing names. New airframes are added as
JSON and join the evaluation with no notice.

---

## Scoring

Per run, **lower is better**:

```
score = 1.0   * tracking_error_mean      (mean |commanded − actual|)
      + 1.0   * settling_time_mean_s     (mean time to settle after each step)
      + 1e-05 * mean_thrust_n            (propellant efficiency)
      + 1.0   * mean_ang_rate_rad_s      (rotation nobody asked for)
```

Your `aggregate` is the weighted mean over **24 runs** = 4 rockets × 2 modes × 3 seeds, each 60
simulated seconds. Per-rocket and per-mode weights default to 1.0.

A **failed run costs 1000.0**, far worse than any real score. Crashing on the hard vehicle is not a
strategy.

**The weights are data, published on every leaderboard response, and will be retuned.** They were
guessed before any submission existed. Do not overfit to them — re-weighting is a re-read of stored
results, so your archived components stay valid when the policy changes.

---

## Determinism is mandatory

The host re-runs a configuration **in a second process** and compares the state hash. Disagree and
you are marked `nondeterministic` and **refused a rank**, even with 24/24 successful runs.

So: no clocks, no `rand()` without a fixed seed, no thread IDs, no pointer values, no uninitialised
memory, nothing keyed on the process. Clear all state in `rf_reset`. Static state that survives an
episode is the usual culprit.

---

## Limits

Per job — one `(rocket, mode, seed)`:

| | |
| --- | --- |
| Wall clock | **10 s**, then `SIGKILL` to the process group |
| CPU time | 5 s soft / 6 s hard (`RLIMIT_CPU`) |
| Address space | 1 GiB · File size 16 MiB · Open files 64 · Processes 64 |
| stdout + stderr | 4 MiB, then the job is killed |

For scale: the reference controller finishes a 60 s run in about **20 ms** — 500× under the
deadline, about 1.6 ms per `rf_resolve`. Hitting the limit means something is wrong, not tight.

Environment is replaced (not extended), cwd is a scratch dir, `stdin` is `/dev/null`, and network
access is removed best-effort. Compiles get 60 s wall / 45 s CPU.

---

## Reading your results

- `GET /api/leaderboard` — every entry's summary: `aggregate`, `per_rocket`, `per_mode`, `status`,
  `ranked`, `deterministic`, `runs_ok` / `runs_failed`.
- `GET /api/submissions/<id>` — your full record, including all 24 runs with per-run `metrics`,
  `seed`, `score`, `state_hash` and `wall_seconds`.

The diagnostic field is **`settled_fraction`** with `steps` / `steps_settled`: how often you
actually converged before the next command arrived. A low value means you are chasing the sequence,
not tracking it.

Reproduce any run exactly, offline:

```sh
./build/rocketfight_bench --rocket=norcs --mode=accel --seed=<seed> --seconds=60
./build/rocketfight_runner --rocket=norcs --mode=accel --seed=<seed> \
                           --library=$PWD/mine.so --pretty
```

Same seed and rocket give byte-identical results, and `state_hash` proves it. Note results are
public: any client can read any submission's per-run detail.

---

## Fly your own submission by hand

Point the game at your submissions and cycle through them with the **D-pad** (or `,` / `.`):

```sh
./build/rocketfight --submissions=server-data          # the server's store
./build/rocketfight --submissions=/path/to/my/libs     # or loose .so files
```

The built-in fly-by-wire is always the first entry, so there is always a way back to something
known to work, and the HUD names whichever is flying. Press `B` while yours is selected and the
benchmark drives **it**, on the same seeded sequence the leaderboard uses — the most direct way to
see *why* a number is what it is. Changing vehicle re-initialises your controller against the new
airframe.

**The game does not sandbox you.** The server evaluates in a separate supervised process because a
submission can hang or crash; loading one into the game puts it in the game's own process, where
nothing can kill it. Your own buggy controller will take the window down. That is fine for
development and is exactly why the server does it differently.

---

## Fine print

- `--seconds` is capped at 300 locally. The benchmark deliberately never checks the 1000 km world
  bound, so that every submission integrates over an identical window; past ~450 s vehicles drift
  outside it. Scored runs are 60 s, so this cannot affect you.
- A `.so` path passed to the runner must contain a `/`, or `dlopen` searches the loader path.
- The sandbox is hardening against buggy and careless code, **not** a security boundary. Do not
  treat the absence of a wall as an invitation.
