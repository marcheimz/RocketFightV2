/* ==========================================================================
 * A complete submission in plain C99, with no dependency on anything in this
 * repository except the one ABI header.
 *
 * It exists to prove the boundary really is language-agnostic. If this file
 * needed a C++ compiler, or a header out of src/, or a link against rf_control,
 * then the "shared object with a C ABI" story would be a story about C++
 * plugins wearing a disguise. It needs none of those: `cc -shared -fPIC` and an
 * -I pointing at include/ is the entire build.
 *
 * As a controller it is deliberately ordinary -- a heading profile plus a
 * throttle gate, roughly the shape of the built-in fly-by-wire with none of its
 * care about ignition dead time. It flies all four shipped airframes, it does
 * not hardcode any of them, and it scores worse than the reference. Beating it
 * should be easy; that is the point of shipping it.
 * ========================================================================== */

#include "rocketfight/flybywire_abi.h"

#include <math.h>

#define PI 3.14159265358979323846

/* --- the vehicle, classified once in rf_init ---------------------------- */

typedef struct {
    int    count;

    /* Torque per newton with the nozzle straight. Sign is the sign of the
     * rotation it causes: positive is counter-clockwise. Zero for a thruster
     * whose line of action passes through the centre of mass. */
    double lever[RF_MAX_THRUSTERS];

    /* Torque per newton per radian of nozzle deflection. */
    double gimbal_arm[RF_MAX_THRUSTERS];

    /* How much this thruster pushes the hull along its nose, per newton. */
    double forwardness[RF_MAX_THRUSTERS];

    double inertia;
    double mass;

    /* Best torque available in each direction from placement alone, and from
     * deflecting nozzles at full throttle. Derived, never assumed: `norcs` has
     * no attitude thrusters at all and gets zero here, which is what tells the
     * controller it must light an engine before it can turn. */
    double torque_ccw;
    double torque_cw;
    double torque_gimbal;

    double max_forward_accel;
} Vehicle;

static Vehicle g_v;

/* --- small helpers, because C brings none of this ----------------------- */

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Shortest signed angle from `from` to `to`, in (-pi, pi]. Getting this wrong
 * is the classic way to make a rocket take the long way round. */
static double wrap_pi(double a) {
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

/* --- entry points -------------------------------------------------------- */

RF_EXPORT int rf_abi_version(void) { return RF_ABI_VERSION; }

RF_EXPORT int rf_init(const RfRocketSpec* spec) {
    int i;
    if (spec == 0 || spec->thruster_count <= 0) return 1;
    if (spec->mass <= 0.0 || spec->inertia <= 0.0) return 1;

    g_v.count   = spec->thruster_count < RF_MAX_THRUSTERS ? spec->thruster_count
                                                          : RF_MAX_THRUSTERS;
    g_v.mass    = spec->mass;
    g_v.inertia = spec->inertia;

    g_v.torque_ccw        = 0.0;
    g_v.torque_cw         = 0.0;
    g_v.torque_gimbal     = 0.0;
    g_v.max_forward_accel = 0.0;

    for (i = 0; i < g_v.count; ++i) {
        const RfThruster* t  = &spec->thrusters[i];
        const double      dx = cos(t->direction);
        const double      dy = sin(t->direction);

        /* cross(mount, direction) -- the moment arm. */
        g_v.lever[i] = t->mount.x * dy - t->mount.y * dx;
        /* cross(mount, perp(direction)) -- what a deflected nozzle buys. */
        g_v.gimbal_arm[i]  = t->mount.x * (-dx) - t->mount.y * dy;
        g_v.forwardness[i] = dx;

        if (g_v.lever[i] > 0.0) {
            g_v.torque_ccw += g_v.lever[i] * t->max_thrust;
        } else {
            g_v.torque_cw += -g_v.lever[i] * t->max_thrust;
        }
        if (t->max_gimbal > 0.0) {
            g_v.torque_gimbal +=
                fabs(g_v.gimbal_arm[i]) * t->max_thrust * sin(t->max_gimbal);
        }
        if (dx > 0.0) g_v.max_forward_accel += dx * t->max_thrust / spec->mass;
    }
    return 0;
}

/* Nothing accumulates: the rate profile below reads angular velocity out of the
 * observation rather than integrating an error, so there is nothing to clear.
 * The function still has to exist -- a missing symbol is a load failure. */
RF_EXPORT void rf_reset(void) {}

RF_EXPORT void rf_resolve(const RfIntent* intent, const RfObservation* obs,
                          RfControlInput* out) {
    int    i;
    double ax, ay, want_mag, want_dir;
    double heading_err, brake_alpha, rate_target, torque_cmd, torque_avail;
    double throttle, align;

    if (intent == 0 || obs == 0 || out == 0) return;
    if (g_v.count <= 0) return;

    /* 1. What acceleration is being asked for, in world frame and in m/s^2.
     *    Acceleration mode arrives as a fraction of the vehicle's own maximum,
     *    which is what makes one controller portable across airframes. */
    if (intent->mode == RF_MODE_VELOCITY) {
        /* A proportional law: close the velocity gap in about half a second,
         * saturated at what the vehicle can actually produce. */
        ax = 2.0 * (intent->vector.x - obs->vel.x);
        ay = 2.0 * (intent->vector.y - obs->vel.y);
    } else {
        ax = intent->vector.x * obs->max_accel;
        ay = intent->vector.y * obs->max_accel;
    }

    want_mag = sqrt(ax * ax + ay * ay);
    if (want_mag > obs->max_accel && obs->max_accel > 0.0) {
        ax *= obs->max_accel / want_mag;
        ay *= obs->max_accel / want_mag;
        want_mag = obs->max_accel;
    }

    /* Below a nudge, stop flying. Chasing an error smaller than the smallest
     * available correction is a limit cycle wearing a control loop's clothes. */
    if (want_mag < 0.05) {
        /* Still worth killing residual spin, or the vehicle tumbles between
         * commands and the attitude-wander metric charges for all of it. */
        want_dir    = obs->angle;
        heading_err = 0.0;
    } else {
        want_dir    = atan2(ay, ax);
        heading_err = wrap_pi(want_dir - obs->angle);
    }

    /* 2. Attitude, as a deceleration profile rather than a PD term. Ask only
     *    for the rate the vehicle can still stop from before it arrives:
     *    omega = sqrt(2 * alpha * |error|). A PD controller with saturating
     *    thrusters commits to rates it cannot brake out of and overshoots every
     *    large heading change. */
    torque_avail = g_v.torque_ccw < g_v.torque_cw ? g_v.torque_ccw : g_v.torque_cw;
    if (torque_avail <= 0.0) {
        /* No attitude thrusters: everything this vehicle has comes from
         * deflecting a lit engine, and only part of it can be counted on, since
         * throttle may drop at any moment. */
        torque_avail = 0.35 * g_v.torque_gimbal;
    }
    brake_alpha = 0.6 * torque_avail / g_v.inertia;

    rate_target = 0.0;
    if (brake_alpha > 0.0 && fabs(heading_err) > 0.0) {
        rate_target = sqrt(2.0 * brake_alpha * fabs(heading_err));
        if (rate_target > 2.0) rate_target = 2.0;
        if (heading_err < 0.0) rate_target = -rate_target;
    }

    /* First-order lag onto that rate. tau is the time constant, not a gain, so
     * it stays meaningful if the control rate ever changes. */
    torque_cmd = g_v.inertia * (rate_target - obs->ang_vel) / 0.35;

    /* Deadband. With a thrust floor the smallest correction available is not
     * small, and firing it at an error smaller than itself makes things worse. */
    if (fabs(heading_err) < 0.01 && fabs(obs->ang_vel) < 0.03) torque_cmd = 0.0;

    /* 3. Throttle. Do not push until the nose is roughly where the push should
     *    go, or the vehicle accelerates confidently in the wrong direction. */
    align    = want_mag > 0.0 ? cos(heading_err) : 0.0;
    throttle = 0.0;
    if (align > 0.5 && g_v.max_forward_accel > 0.0) {
        throttle = clampd(want_mag / g_v.max_forward_accel, 0.0, 1.0);
    }
    /* A vehicle with no attitude thrusters has to burn a little in order to
     * steer at all, so give it a floor whenever it wants to turn. */
    if (g_v.torque_ccw <= 0.0 && g_v.torque_cw <= 0.0 && fabs(torque_cmd) > 0.0) {
        if (throttle < 0.25) throttle = 0.25;
    }

    /* 4. Hand out the demands. */
    for (i = 0; i < g_v.count; ++i) {
        double level = 0.0;

        if (g_v.forwardness[i] > 0.7) level = throttle;

        /* Attitude thrusters: fire the ones whose torque has the sign wanted.
         * On a symmetric layout that is an opposed pair, so the forces cancel
         * and only the couple survives -- geometry, not arrangement. */
        if (fabs(g_v.lever[i]) > 1e-9 && torque_cmd != 0.0) {
            if ((torque_cmd > 0.0) == (g_v.lever[i] > 0.0)) {
                const double share =
                    fabs(torque_cmd) / (torque_cmd > 0.0 ? g_v.torque_ccw : g_v.torque_cw);
                const double want = clampd(share, 0.0, 1.0);
                if (want > level) level = want;
            }
        }

        out->level[i] = clampd(level, 0.0, 1.0);

        /* Gimbal, as a fraction of max deflection. Sign flips with the arm, so
         * a nozzle mounted the other way round still turns the right way. */
        if (g_v.gimbal_arm[i] != 0.0 && g_v.torque_gimbal > 0.0) {
            const double frac = torque_cmd / g_v.torque_gimbal;
            out->gimbal[i]    = clampd(g_v.gimbal_arm[i] > 0.0 ? frac : -frac, -1.0, 1.0);
        }
    }

    out->fire = intent->fire;
}

RF_EXPORT void rf_shutdown(void) {}
