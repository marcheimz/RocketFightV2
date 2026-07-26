/* ==========================================================================
 * Every way a submission can be wrong before it has flown a single tick, in
 * one file, selected by a -D at build time.
 *
 * One file rather than six because the interesting content is what is *absent*
 * or *wrong* in each variant, and six near-identical files make that difference
 * invisible. The build rules are in CMakeLists.txt under "ABI failure-path
 * fixtures"; the assertions are in tests/test_submission.cpp.
 *
 * These are fixtures, not examples. They are built into the test tree and never
 * shipped as submissions.
 * ========================================================================== */

/* nanosleep is POSIX, and -std=c99 is strict ISO: without this the declaration
 * is hidden and the RF_FIXTURE_SLEEP variant compiles to an implicit call. */
#define _POSIX_C_SOURCE 199309L

#include "rocketfight/flybywire_abi.h"

#include <time.h>     /* nanosleep, for the RF_FIXTURE_SLEEP variant */
#include <unistd.h>   /* getpid,   for the RF_FIXTURE_NONDETERMINISTIC variant */

#if defined(RF_FIXTURE_WRONG_VERSION)
/* Reports a version this host does not serve. The host must refuse before it
 * calls anything else, because a struct-layout mismatch does not announce
 * itself -- it reads plausible garbage out of the middle of an observation and
 * looks like a controller that flies badly. */
RF_EXPORT int rf_abi_version(void) { return RF_ABI_VERSION + 1000; }
#else
RF_EXPORT int rf_abi_version(void) { return RF_ABI_VERSION; }
#endif

#if defined(RF_FIXTURE_INIT_DECLINES)
/* A controller entitled to say "not this vehicle". Nonzero is a decline, not a
 * crash, and the host records the episode as a failure and moves on. */
RF_EXPORT int rf_init(const RfRocketSpec* spec) {
    (void)spec;
    return 7;
}
#else
RF_EXPORT int rf_init(const RfRocketSpec* spec) {
    (void)spec;
    return 0;
}
#endif

RF_EXPORT void rf_reset(void) {}

#if !defined(RF_FIXTURE_NO_RESOLVE)
#if defined(RF_FIXTURE_SEGFAULT)
/* Loads, initialises, and then dereferences a null pointer on the first control
 * step. Nothing before this point could have caught it, which is the whole
 * argument for the process boundary: the host cannot validate its way out of a
 * wild store, it can only survive one. */
RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    (void)intent;
    (void)obs;
    (void)out;
    volatile int* wild = (volatile int*)0;
    *wild              = 1;
}
#elif defined(RF_FIXTURE_SLEEP)
/* Hangs without burning a cycle. This is the case no rlimit catches: RLIMIT_CPU
 * counts CPU time and this consumes none, RLIMIT_AS counts memory and this
 * allocates none. Only a wall-clock deadline in the *parent* ends it, which is
 * why the supervisor has one in addition to the rlimits rather than instead of
 * them. */
RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    (void)intent;
    (void)obs;
    (void)out;
    for (;;) {
        struct timespec long_nap = {3600, 0};
        (void)nanosleep(&long_nap, (struct timespec*)0);
    }
}
#elif defined(RF_FIXTURE_NONDETERMINISTIC)
/* Flies -- badly, but it flies, produces a state hash, and scores. What it also
 * does is key one thruster level off its own process id, which is the cheapest
 * honest stand-in for the real offenders: a clock, /dev/urandom, an
 * uninitialised variable, a pointer value, a hash-map iteration order.
 *
 * The point of the fixture is *where* this is caught. Inside one process it
 * reproduces perfectly, so a determinism check that ran both passes in the same
 * address space would call it clean. The host re-runs the first seed of every
 * (rocket, mode) pair in a brand-new process, and that is what makes the two
 * hashes disagree. One entry like this makes every comparison on the board
 * meaningless, so it is recorded and shown but never ranked. */
RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    (void)intent;
    const double jitter = (double)(getpid() % 97) / 97.0;
    int          i;
    for (i = 0; i < obs->thruster_count; ++i) {
        out->level[i] = jitter;
    }
}
#elif defined(RF_FIXTURE_NAN)
/* Answers with garbage rather than refusing. NaN in a thruster level would
 * poison the world state and therefore the state hash, so the host replaces
 * non-finite values with zero before the physics ever sees them -- and the test
 * asserts the sanitised value, not the fact that it did not crash. */
RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    (void)intent;
    (void)obs;
    /* Computed through volatiles rather than written as 0.0/0.0: a literal
     * division by zero is a constraint violation the compiler is entitled to
     * reject, and a fixture that does not build tests nothing. */
    static volatile double zero = 0.0;
    static volatile double one  = 1.0;
    const double           nan  = zero / zero;
    const double           inf  = one / zero;
    int                    i;
    for (i = 0; i < RF_MAX_THRUSTERS; ++i) {
        out->level[i]  = nan;
        out->gimbal[i] = inf; /* non-finite, and out of range besides */
    }
    out->fire = 1;
}
#else
/* The control: a library that is entirely correct and does nothing. Every other
 * variant differs from this one in exactly one way, which is what makes a
 * failing assertion point at a cause. */
RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    (void)intent;
    (void)obs;
    (void)out;
}
#endif
#endif /* !RF_FIXTURE_NO_RESOLVE */

RF_EXPORT void rf_shutdown(void) {}
