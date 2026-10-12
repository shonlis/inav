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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "common/vector.h"

// ---- Watchdog -------------------------------------------------------------
#define MAX_DEAD_RECKONING_TIME_MS      20000   // Hard limit on blind flight since last trusted GPS fix
#define DR_MAX_ERROR_CM                 5000.0f // Modelled position error that also trips the watchdog [cm]
#define DR_GPS_RECOVERY_MS              2000    // GPS must be continuously valid this long to be trusted again

// ---- Integrator -----------------------------------------------------------
#define DR_VEL_DECAY_TAU_S              5.0f    // Velocity leak time constant (HPF corner = 1/(2*pi*tau) ~ 0.03 Hz)
#define DR_ACC_DEADBAND_CMSS            15.0f   // Residual accel below this is treated as bias/noise [cm/s^2]
#define DR_MAX_VEL_XY_CMS               2500.0f // Physical horizontal speed clamp [cm/s]
#define DR_MAX_VEL_Z_CMS                800.0f  // Physical vertical speed clamp [cm/s]
#define DR_MAX_DT_S                     0.1f    // Reject integration steps longer than this (scheduler stall)

// ---- Error growth model: err(t) = eph0 + VEL_ERR*t + 0.5*ACC_ERR*t^2 -----
#define DR_VEL_ERR_CMS                  100.0f  // Unobservable velocity (steady wind drift) [cm/s]
#define DR_ACC_ERR_CMSS                 10.0f   // Residual bias + ~0.5 deg tilt error [cm/s^2]
#define DR_ACC_ERR_NO_HEADING_CMSS      50.0f   // Same, once yaw is no longer trustworthy [cm/s^2]

// EXPERIMENTAL: publish DR position to the nav controller while the smart failsafe is in control.
// Off by default: the velocity leak forgets steady cruise speed, so a position loop closed on
// this estimate under-reads speed and overshoots. Validate in SITL before enabling.
#define DR_PUBLISH_TO_NAV               0

typedef struct deadReckoningState_s {
    bool        seeded;             // Had at least one trusted GPS-based estimate
    bool        active;             // Currently dead-reckoning (GPS not trusted)
    bool        gpsGood;            // GPS currently reports a valid fix
    bool        headingLost;        // Yaw became invalid at some point in this DR session
    timeMs_t    gpsGoodSinceMs;     // Start of the current continuous valid-fix window
    timeMs_t    lastValidGpsMs;     // Last time GPS was trusted (watchdog reference)
    fpVector3_t pos;                // NEU, cm, same origin as posEstimator.est.pos
    fpVector3_t vel;                // NEU, cm/s
    float       seedEphCm;          // Estimator EPH at the moment GPS was lost
    float       errorBoundCm;       // Modelled horizontal error
    uint16_t    activations;
} deadReckoningState_t;

extern bool drift_too_high;

void deadReckoningInit(void);
void updateDeadReckoningPosition(float dt);
bool deadReckoningIsActive(void);
bool deadReckoningDriftTooHigh(void);
bool deadReckoningShouldPublish(void);
const deadReckoningState_t *getDeadReckoningState(void);
