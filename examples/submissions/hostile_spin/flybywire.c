/* ==========================================================================
 * A deliberately hostile submission. It is here to be killed.
 *
 * `rf_resolve` never returns. That is the failure mode a submission server has
 * no in-process answer to: there is no portable way to abort a function call
 * that has decided not to come back, no timeout on a `for(;;)`, and no signal
 * the host can raise inside its own process that leaves the rest of it usable.
 * The only containment is a process boundary with a supervisor on the far side
 * of it, which is exactly what src/server/Sandbox.cpp is.
 *
 * What is being demonstrated, precisely:
 *
 *   - the runner process hangs on the very first control step;
 *   - the supervisor kills it -- SIGKILL, to the whole process group;
 *   - the job is recorded as a failed run with a reason, per (rocket, mode,
 *     seed), and charged the failure penalty;
 *   - the remaining jobs still run, so the submission gets a complete row;
 *   - the server answers requests throughout and is still up afterwards.
 *
 * This one burns CPU while it hangs, which means two independent limits are
 * racing to end it: RLIMIT_CPU (5 s soft, 6 s hard) and the wall-clock deadline
 * (10 s). On a machine that is not otherwise loaded, CPU time accumulates at
 * roughly wall-clock rate, so RLIMIT_CPU wins and the child dies on SIGXCPU at
 * about 5 s. That is a correct outcome and it is *not* the interesting one: a
 * submission that blocks instead of spinning -- one `nanosleep` loop, one read
 * from a pipe nobody writes to -- consumes no CPU at all and sails past every
 * rlimit there is. The wall-clock deadline is the limit that catches that case,
 * and it is the only one that catches every case, which is why it exists in
 * addition to the rlimits rather than instead of them.
 *
 * Do not copy this file. It scores the failure penalty on every configuration,
 * which is by construction worse than any real controller can do.
 * ========================================================================== */

#include "rocketfight/flybywire_abi.h"

RF_EXPORT int rf_abi_version(void) { return RF_ABI_VERSION; }

RF_EXPORT int rf_init(const RfRocketSpec* spec) {
    (void)spec;
    /* Loading and initialising are both fine. The hang is in the hot path, on
     * purpose: a submission that hung in rf_init would be caught by a host that
     * only supervised rf_resolve, and this one would not. */
    return 0;
}

RF_EXPORT void rf_reset(void) {}

/* `volatile` so the loop survives -O2. A compiler is entitled to delete an
 * infinite loop with no side effects, and a hostile example the optimiser
 * optimises away proves nothing. */
static volatile unsigned long long g_spin;

RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    (void)intent;
    (void)obs;
    (void)out;
    for (;;) {
        g_spin = g_spin + 1u;
    }
}

RF_EXPORT void rf_shutdown(void) {}
