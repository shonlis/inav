/*
 * This file is part of INAV.
 *
 * INAV is free software. You can redistribute this software
 * and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * INAV is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * IMU-only Dead Reckoning fallback position estimator.
 *
 * While GPS is trusted this module simply tracks INAV's fused estimate
 * (posEstimator.est) so it always holds a good seed. When GPS is lost it
 * free-runs on posEstimator.imu.accelNEU, which is already:
 *   - rotated body->NEU by the attitude quaternion,
 *   - corrected by the in-flight learned accelerometer bias,
 *   - gravity-compensated with the calibrated 1G value,
 *   - in cm/s^2.
 *
 * Integration chain per axis (semi-implicit, unconditionally stable):
 *
 *   a'     = softDeadband(a)                       // reject residual bias/noise
 *   v[k+1] = v[k] * 1/(1 + dt/tau) + a' * dt       // leaky integrator = 1st-order HPF
 *   p[k+1] = p[k] + 0.5 * (v[k] + v[k+1]) * dt     // trapezoidal
 *
 * The leak makes V(s) = A(s) / (s + 1/tau): a constant accelerometer bias b
 * produces a bounded velocity error b*tau instead of b*t, and position error
 * grows ~linearly (b*tau*t) instead of quadratically (0.5*b*t^2).
 * The price: true constant-velocity motion is forgotten with time constant tau.
 *
 * Drift watchdog: a time limit since the last trusted GPS fix plus a
 * modelled error bound. Either one latches drift_too_high until GPS has been
 * continuously valid for DR_GPS_RECOVERY_MS.
 *
 * Non-blocking, O(1), no dynamic allocation. Called at estimator loop rate.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/navigation_pos_estimator_private.h"
#include "navigation/dead_reckoning.h"

#include "sensors/acceleration.h"
#include "sensors/sensors.h"

extern navigationPosEstimator_t posEstimator;

bool drift_too_high = true;     // No seed yet -> no dead reckoning possible

static deadReckoningState_t dr;

/* ---- Helpers ------------------------------------------------------------ */

// Continuous soft deadband: 0 inside +-db, shifted linear outside (no step at the edge)
static float drSoftDeadband(float value, float db)
{
    if (value > db) {
        return value - db;
    }
    if (value < -db) {
        return value + db;
    }
    return 0.0f;
}

// Radial soft deadband for the horizontal vector (keeps direction, isotropic)
static void drSoftDeadbandXY(float *x, float *y, float db)
{
    const float mag = calc_length_pythagorean_2D(*x, *y);
    if (mag <= db) {
        *x = 0.0f;
        *y = 0.0f;
        return;
    }
    const float scale = (mag - db) / mag;
    *x *= scale;
    *y *= scale;
}

static bool drGpsIsValidNow(void)
{
    // STATE(GPS_FIX) excludes INAV's *estimated* fix, so a fix synthesised
    // by GPS fix estimation never counts as a real fix here
    return sensors(SENSOR_GPS) && STATE(GPS_FIX) &&
           (posEstimator.flags & EST_GPS_XY_VALID) &&
           (posEstimator.est.eph < positionEstimationConfig()->max_eph_epv);
}

static void drSeedFromEstimator(void)
{
    dr.pos = posEstimator.est.pos;
    dr.vel = posEstimator.est.vel;
    dr.seedEphCm = posEstimator.est.eph;
    dr.seeded = true;
}

/* ---- Integrator --------------------------------------------------------- */

static void drIntegrate(float dt)
{
    float ax = posEstimator.imu.accelNEU.x;
    float ay = posEstimator.imu.accelNEU.y;
    float az = posEstimator.imu.accelNEU.z;

    // Clipped samples are saturated garbage - coast on velocity only
    if (accIsClipped() || !isImuReady()) {
        ax = ay = az = 0.0f;
    }

    drSoftDeadbandXY(&ax, &ay, DR_ACC_DEADBAND_CMSS);
    az = drSoftDeadband(az, DR_ACC_DEADBAND_CMSS);

    // Leak factor: 1/(1 + dt/tau) == backward-Euler of dv/dt = -v/tau, stable for any dt
    const float decay = 1.0f / (1.0f + dt / DR_VEL_DECAY_TAU_S);

    const fpVector3_t velPrev = dr.vel;

    // First integration: acceleration -> velocity (leaky)
    dr.vel.x = dr.vel.x * decay + ax * dt;
    dr.vel.y = dr.vel.y * decay + ay * dt;
    dr.vel.z = dr.vel.z * decay + az * dt;

    // Physical limits: a runaway integrator must never exceed what the airframe can do
    const float velXY = calc_length_pythagorean_2D(dr.vel.x, dr.vel.y);
    if (velXY > DR_MAX_VEL_XY_CMS) {
        const float scale = DR_MAX_VEL_XY_CMS / velXY;
        dr.vel.x *= scale;
        dr.vel.y *= scale;
    }
    dr.vel.z = constrainf(dr.vel.z, -DR_MAX_VEL_Z_CMS, DR_MAX_VEL_Z_CMS);

    // Second integration: velocity -> position (trapezoidal)
    dr.pos.x += 0.5f * (velPrev.x + dr.vel.x) * dt;
    dr.pos.y += 0.5f * (velPrev.y + dr.vel.y) * dt;
    dr.pos.z += 0.5f * (velPrev.z + dr.vel.z) * dt;
}

/* ---- Watchdog ----------------------------------------------------------- */

static void drUpdateWatchdog(timeMs_t nowMs)
{
    if (!dr.seeded) {
        dr.errorBoundCm = INFINITY;
        drift_too_high = true;
        return;
    }

    const timeMs_t blindMs = (timeMs_t)(nowMs - dr.lastValidGpsMs);
    const float t = MS2S(blindMs);

    if (!isImuHeadingValid()) {
        // MPU6500 has no magnetometer: without GPS course updates yaw drifts,
        // and every degree of yaw error rotates accelNEU in the XY plane
        dr.headingLost = true;
    }
    const float accErr = dr.headingLost ? DR_ACC_ERR_NO_HEADING_CMSS : DR_ACC_ERR_CMSS;

    // Horizontal error model: seed + unobservable velocity + residual accel (quadratic term)
    dr.errorBoundCm = dr.seedEphCm + DR_VEL_ERR_CMS * t + 0.5f * accErr * t * t;

    // Latch: once tripped, only a GPS recovery (handled in update) clears it
    if ((blindMs > MAX_DEAD_RECKONING_TIME_MS) || (dr.errorBoundCm > DR_MAX_ERROR_CM)) {
        drift_too_high = true;
    }
}

/* ---- Public API --------------------------------------------------------- */

void deadReckoningInit(void)
{
    vectorZero(&dr.pos);
    vectorZero(&dr.vel);
    dr.seeded = false;
    dr.active = false;
    dr.gpsGood = false;
    dr.headingLost = false;
    dr.gpsGoodSinceMs = 0;
    dr.lastValidGpsMs = 0;
    dr.seedEphCm = 0.0f;
    dr.errorBoundCm = INFINITY;
    dr.activations = 0;
    drift_too_high = true;
}

void updateDeadReckoningPosition(float dt)
{
    const timeMs_t nowMs = millis();
    const bool gpsValidNow = drGpsIsValidNow();

    // Track the continuous valid-fix window (rejects single good samples during jamming)
    if (gpsValidNow && !dr.gpsGood) {
        dr.gpsGoodSinceMs = nowMs;
    }
    dr.gpsGood = gpsValidNow;

    const bool gpsTrusted = gpsValidNow &&
                            (!dr.active || ((timeMs_t)(nowMs - dr.gpsGoodSinceMs) >= DR_GPS_RECOVERY_MS));

    if (gpsTrusted) {
        // TRACKING: follow the fused estimate, keep the seed fresh, clear the watchdog
        drSeedFromEstimator();
        dr.lastValidGpsMs = nowMs;
        dr.active = false;
        dr.headingLost = false;
        dr.errorBoundCm = dr.seedEphCm;
        drift_too_high = false;
        return;
    }

    if (!dr.seeded) {
        // Never had a trusted GPS solution - nothing to dead-reckon from
        drUpdateWatchdog(nowMs);
        return;
    }

    if (!dr.active) {
        // Transition TRACKING -> DEAD RECKONING. dr.pos/vel already hold the
        // last trusted fused state from the previous call.
        dr.active = true;
        dr.headingLost = false;
        dr.activations++;
    }

    // Integrate only on sane steps; the watchdog is time-based and always runs
    if ((dt > 0.0f) && (dt <= DR_MAX_DT_S)) {
        drIntegrate(dt);
    }

    drUpdateWatchdog(nowMs);
}

bool deadReckoningIsActive(void)
{
    return dr.active;
}

bool deadReckoningDriftTooHigh(void)
{
    return drift_too_high;
}

bool deadReckoningShouldPublish(void)
{
#if DR_PUBLISH_TO_NAV
    // Only inside an anti-jamming failsafe, only while the drift budget holds
    return dr.active && !drift_too_high && failsafeSmartIsControlling();
#else
    return false;
#endif
}

const deadReckoningState_t *getDeadReckoningState(void)
{
    return &dr;
}
