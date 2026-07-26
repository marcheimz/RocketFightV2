/* ==========================================================================
 * RocketFight fly-by-wire submission ABI, version 1.
 *
 * This is the whole contract between the evaluation host and a submitted
 * controller. Include it, implement the five entry points, build a shared
 * object, submit it. Nothing else in this repository is part of the contract.
 *
 * It is deliberately plain C89-clean C: fixed-size structs of `double`, `int`
 * and one `unsigned long long`, no bitfields, no enums in struct fields, no
 * pointers inside structs, no callbacks, no ownership transfer. A C++, Rust,
 * Zig or hand-written-assembly submission is exactly as much of a first-class
 * citizen as a C one, because none of the host's C++ ever crosses this line --
 * no classes, no std:: types, no exceptions, no allocator, no RTTI.
 *
 * Consequences worth stating rather than discovering:
 *
 *   - An exception, panic or unwind must NOT escape any of these functions.
 *     Unwinding through a C frame is undefined behaviour, and the host cannot
 *     catch what it cannot see. Catch at the boundary. (The host runs every
 *     evaluation in its own process, so a submission that ignores this crashes
 *     itself and scores "crashed" -- it does not take the server with it.)
 *   - `RfControlInput` is written by you, read by the host, and owned by the
 *     host's memory. Fill it; do not retain the pointer past the call.
 *   - Everything handed to you is const and valid only for the duration of the
 *     call. Copy anything you want to keep.
 *   - Angles are radians, positions metres, velocities m/s, thrust newtons,
 *     time seconds. Positive rotation is counter-clockwise. Body frame +x is
 *     the nose.
 *
 * DETERMINISM IS PART OF THE CONTRACT. The host runs at least one configuration
 * twice, in two separate processes, and compares a hash of the resulting world
 * state. A submission that reads a clock, `rand()`, a thread id, a pointer
 * value, /dev/urandom, or an uninitialised variable will be flagged as
 * nondeterministic and will not be ranked. Seeded pseudo-randomness is fine;
 * seed it from something inside this header, not from the environment.
 * ========================================================================== */

#ifndef ROCKETFIGHT_FLYBYWIRE_ABI_H
#define ROCKETFIGHT_FLYBYWIRE_ABI_H

/* Bumped whenever a struct layout or a function signature changes. The host
 * refuses to load a library whose rf_abi_version() does not equal the value it
 * was built with, because the alternative is reading the wrong bytes out of a
 * struct and blaming the controller for it. */
#define RF_ABI_VERSION 1

/* Matches rf::kMaxThrusters. The arrays below are always this long; only the
 * first `thruster_count` entries are meaningful, and the rest are zeroed. */
#define RF_MAX_THRUSTERS 16

/* Values of RfIntent::mode. Plain macros rather than an enum: the underlying
 * type of a C enum is implementation-defined, and this struct crosses a
 * language boundary. */
#define RF_MODE_ACCELERATION 0
#define RF_MODE_VELOCITY     1

/* Put the five entry points in the dynamic symbol table even when the rest of
 * the library is built with -fvisibility=hidden, which is what a submission
 * that statically links a helper library wants. */
#if defined(_WIN32)
#define RF_EXPORT __declspec(dllexport)
#elif defined(__GNUC__)
#define RF_EXPORT __attribute__((visibility("default")))
#else
#define RF_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RfVec2 {
    double x;
    double y;
} RfVec2;

/* One thruster's immutable definition. Everything that differs between vehicles
 * is here, and a controller that reads it flies every airframe -- including
 * airframes added after the controller was written. Hardcoding a layout is the
 * one reliable way to score badly on the rockets you have not seen. */
typedef struct RfThruster {
    /* Where it is bolted to the hull, body frame, metres from the centre of
     * mass. This is what produces torque: a thruster whose line of action
     * passes through the centre of mass produces none. */
    RfVec2 mount;

    /* Direction of the force applied *to the hull*, body frame, radians. A tail
     * engine pushes the hull forward, so its direction is 0 even though the
     * exhaust goes the other way. */
    double direction;

    /* Once lit it cannot be throttled below min_thrust. It is off, or it is
     * somewhere in [min_thrust, max_thrust], and nothing in between at any
     * price. This is what makes allocation partly discrete. */
    double min_thrust;
    double max_thrust;

    /* Seconds between being commanded on and producing anything at all. */
    double ignition_time;

    /* Newtons per second the thrust can move once burning. */
    double ramp_up;
    double ramp_down;

    /* Radians of deflection either side of straight; 0 means a fixed nozzle.
     * gimbal_rate is rad/s of slew -- a nozzle travels, it does not snap, so
     * reversing one takes real time. */
    double max_gimbal;
    double gimbal_rate;
} RfThruster;

/* The vehicle. Handed to rf_init once per episode and constant thereafter. */
typedef struct RfRocketSpec {
    double mass;    /* kg */
    double length;  /* m, nose to tail */
    double width;   /* m */
    double inertia; /* kg*m^2 about the centre of mass; precomputed, since every
                     * controller needs it and deriving it twice invites the two
                     * derivations to disagree. */

    int        thruster_count;
    RfThruster thrusters[RF_MAX_THRUSTERS];
} RfRocketSpec;

/* What a thruster is doing *now*, which is not what it was last told to do.
 *
 * This is the half of the observation that makes the problem solvable. A
 * controller that only knows its own last command is flying blind through its
 * own ignition delays and ramp rates: it keeps re-commanding a correction that
 * is already on its way, and overshoots every time. */
typedef struct RfThrusterState {
    double thrust;         /* N actually being produced right now */
    double gimbal_angle;   /* rad the nozzle has actually reached */
    double ignition_timer; /* s remaining before it lights; 0 if not igniting */
    int    lit;            /* nonzero once burning, or committed to burning */
} RfThrusterState;

typedef struct RfObservation {
    unsigned long long tick; /* physics ticks since the episode began, 1 kHz */
    double             time; /* seconds; tick * 0.001 */

    RfVec2 pos;     /* world frame, m */
    RfVec2 vel;     /* world frame, m/s */
    double angle;   /* rad, world frame; 0 points along world +x */
    double ang_vel; /* rad/s, positive counter-clockwise */

    /* Derived capability, so a controller can normalise without hardcoding the
     * vehicle it was written for. */
    double max_accel;     /* m/s^2 available along the nose */
    double max_ang_accel; /* rad/s^2 from thruster placement alone; 0 on a
                           * vehicle that can only steer by gimballing */

    int             thruster_count;
    RfThrusterState actuators[RF_MAX_THRUSTERS];
} RfObservation;

/* What the pilot wants, in terms that mention no thrusters. The benchmark's
 * scripted pilot occupies exactly the slot a human gamepad does. */
typedef struct RfIntent {
    int mode; /* RF_MODE_ACCELERATION or RF_MODE_VELOCITY */

    /* Acceleration mode: a world-frame direction whose magnitude is <= 1 and is
     * a fraction of the vehicle's maximum acceleration.
     * Velocity mode: a world-frame *target velocity*, in m/s. */
    RfVec2 vector;

    /* has_facing == 0 means "don't care": where to point is your judgement, and
     * the benchmark scores that judgement. The benchmark never sets it. */
    int    has_facing;
    double facing; /* rad, world frame */

    int fire; /* weapons trigger; wired through, nothing shoots yet */
} RfIntent;

/* Your answer: one demand per thruster. A demand, not a result -- see
 * RfThrusterState for what actually happens to it.
 *
 * The host clamps `level` to [0, 1] and `gimbal` to [-1, 1] and replaces any
 * non-finite value with 0 before handing it to the physics, so a NaN is a
 * silently ignored command rather than a poisoned world hash. Do not rely on
 * that: a clamped command is a command you did not mean. */
typedef struct RfControlInput {
    double level[RF_MAX_THRUSTERS];  /* [0, 1]  throttle demand */
    double gimbal[RF_MAX_THRUSTERS]; /* [-1, 1] fraction of max_gimbal;
                                      * ignored for a fixed nozzle */
    int    fire;
} RfControlInput;

/* ==========================================================================
 * The five entry points. All five must be exported; a missing one is a load
 * failure with a clear message rather than a crash at the first call.
 *
 * Call order, per process, single-threaded, never concurrent:
 *
 *     rf_abi_version()            -- checked first, before anything else runs
 *     rf_init(spec)               -- once; nonzero return aborts the episode
 *     rf_reset()                  -- at least once, before the first resolve
 *     rf_resolve(...) * N         -- at 100 Hz of simulated time
 *     rf_shutdown()               -- once, then the process exits
 *
 * The host runs one episode per process, so global state is fine and there is
 * no handle to thread through. rf_reset() may be called again mid-process if a
 * future host reuses an instance; treat it as "forget everything except the
 * spec".
 * ========================================================================== */

/* Must return RF_ABI_VERSION. Called before the library is trusted with
 * anything else, so it must not depend on rf_init having run. */
RF_EXPORT int rf_abi_version(void);

/* Fit the controller to a vehicle. Solve your allocator here rather than in
 * rf_resolve: this is called once, and rf_resolve is called 6000 times per
 * simulated minute. Return 0 on success, nonzero to decline the vehicle (the
 * host records the episode as a failure and moves on to the next rocket).
 * `spec` points to host memory valid only for this call. */
RF_EXPORT int rf_init(const RfRocketSpec* spec);

/* Clear per-episode state: integrators, filters, previous errors. The spec
 * stays. Called at least once between rf_init and the first rf_resolve. */
RF_EXPORT void rf_reset(void);

/* The job. Called every 10 physics ticks -- 100 Hz of simulated time -- and the
 * result is zero-order held in between, so a command is in force for 10 ms
 * whatever you do next.
 *
 * `*out` is zeroed by the host before the call, so writing nothing is a valid
 * "all thrusters off". Neither pointer may be retained after the call returns.
 * This function must not throw, must not unwind, and must terminate. */
RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out);

/* Release whatever rf_init acquired. The process exits immediately afterwards,
 * so this exists for symmetry and for leak checkers, not because the host needs
 * the memory back. */
RF_EXPORT void rf_shutdown(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ROCKETFIGHT_FLYBYWIRE_ABI_H */
