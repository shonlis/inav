/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#if defined(USE_VTX_CONTROL)
#include "drivers/vtx_common.h"
#endif

#include "io/beeper.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/controlrate_profile.h"
#include "fc/settings.h"

#include "flight/failsafe.h"
#include "flight/jamming_detect.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/dead_reckoning.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

/*
 * Usage:
 *
 * failsafeInit() and failsafeReset() must be called before the other methods are used.
 *
 * failsafeInit() and failsafeReset() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */

static failsafeState_t failsafeState;

PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 3);

PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig,
    .failsafe_delay = SETTING_FAILSAFE_DELAY_DEFAULT,                                   // 0.5 sec
    .failsafe_recovery_delay = SETTING_FAILSAFE_RECOVERY_DELAY_DEFAULT,                 // 0.5 seconds (plus 200ms explicit delay)
    .failsafe_off_delay = SETTING_FAILSAFE_OFF_DELAY_DEFAULT,                           // 20sec
    .failsafe_throttle_low_delay = SETTING_FAILSAFE_THROTTLE_LOW_DELAY_DEFAULT,         // default throttle low delay for "just disarm" on failsafe condition
    .failsafe_procedure = SETTING_FAILSAFE_PROCEDURE_DEFAULT,                           // default full failsafe procedure
    .failsafe_fw_roll_angle = SETTING_FAILSAFE_FW_ROLL_ANGLE_DEFAULT,                   // 20 deg left
    .failsafe_fw_pitch_angle = SETTING_FAILSAFE_FW_PITCH_ANGLE_DEFAULT,                 // 10 deg dive (yes, positive means dive)
    .failsafe_fw_yaw_rate = SETTING_FAILSAFE_FW_YAW_RATE_DEFAULT,                       // 45 deg/s left yaw (left is negative, 8s for full turn)
    .failsafe_stick_motion_threshold = SETTING_FAILSAFE_STICK_THRESHOLD_DEFAULT,
    .failsafe_min_distance = SETTING_FAILSAFE_MIN_DISTANCE_DEFAULT,                     // No minimum distance for failsafe by default
    .failsafe_min_distance_procedure = SETTING_FAILSAFE_MIN_DISTANCE_PROCEDURE_DEFAULT, // default minimum distance failsafe procedure
    .failsafe_mission_delay = SETTING_FAILSAFE_MISSION_DELAY_DEFAULT,                   // Time delay before Failsafe activated during WP mission (s)
#ifdef USE_GPS_FIX_ESTIMATION
    .failsafe_gps_fix_estimation_delay = SETTING_FAILSAFE_GPS_FIX_ESTIMATION_DELAY_DEFAULT, // Time delay before Failsafe activated when GPS Fix estimation is allied
#endif
);

typedef enum {
    FAILSAFE_CHANNEL_HOLD,      // Hold last known good value
    FAILSAFE_CHANNEL_NEUTRAL,   // RPY = zero, THR = zero
} failsafeChannelBehavior_e;

typedef struct {
    bool                        bypassNavigation;
    bool                        forceAngleMode;
    failsafeChannelBehavior_e   channelBehavior[4];
} failsafeProcedureLogic_t;

static const failsafeProcedureLogic_t failsafeProcedureLogic[] = {
    [FAILSAFE_PROCEDURE_AUTO_LANDING] = {
            .forceAngleMode = true,
            .bypassNavigation = false,
            .channelBehavior = {
                FAILSAFE_CHANNEL_NEUTRAL,       // ROLL
                FAILSAFE_CHANNEL_NEUTRAL,       // PITCH
                FAILSAFE_CHANNEL_NEUTRAL,       // YAW
                FAILSAFE_CHANNEL_HOLD           // THROTTLE
            }
    },

    [FAILSAFE_PROCEDURE_DROP_IT] = {
            .bypassNavigation = true,
            .forceAngleMode = true,
            .channelBehavior = {
                FAILSAFE_CHANNEL_NEUTRAL,       // ROLL
                FAILSAFE_CHANNEL_NEUTRAL,       // PITCH
                FAILSAFE_CHANNEL_NEUTRAL,       // YAW
                FAILSAFE_CHANNEL_NEUTRAL        // THROTTLE
            }
    },

    [FAILSAFE_PROCEDURE_RTH] = {
            .bypassNavigation = false,
            .forceAngleMode = true,
            .channelBehavior = {
                FAILSAFE_CHANNEL_NEUTRAL,       // ROLL
                FAILSAFE_CHANNEL_NEUTRAL,       // PITCH
                FAILSAFE_CHANNEL_NEUTRAL,       // YAW
                FAILSAFE_CHANNEL_HOLD           // THROTTLE
            }
    },

    [FAILSAFE_PROCEDURE_NONE] = {
            .bypassNavigation = false,
            .forceAngleMode = false,
            .channelBehavior = {
                FAILSAFE_CHANNEL_HOLD,          // ROLL
                FAILSAFE_CHANNEL_HOLD,          // PITCH
                FAILSAFE_CHANNEL_HOLD,          // YAW
                FAILSAFE_CHANNEL_HOLD           // THROTTLE
            }
    }
};

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void failsafeReset(void)
{
    if (failsafeState.active) {  // can't reset if still active
        return;
    }

    failsafeState.rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY + failsafeConfig()->failsafe_recovery_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.validRxDataReceivedAt = 0;
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = 0;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
    failsafeState.activeProcedure = failsafeConfig()->failsafe_procedure;
    failsafeState.controlling = false;

    failsafeState.lastGoodRcCommand[ROLL] = 0;
    failsafeState.lastGoodRcCommand[PITCH] = 0;
    failsafeState.lastGoodRcCommand[YAW] = 0;
    failsafeState.lastGoodRcCommand[THROTTLE] = 1000;
}

void failsafeInit(void)
{
    failsafeState.events = 0;
    failsafeState.monitoring = false;
    failsafeState.suspended = false;
}

bool failsafeBypassNavigation(void)
{
    return failsafeState.active &&
           failsafeState.controlling &&
           failsafeProcedureLogic[failsafeState.activeProcedure].bypassNavigation;
}

bool failsafeMayRequireNavigationMode(void)
{
    return (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_RTH) ||
           (failsafeConfig()->failsafe_min_distance_procedure == FAILSAFE_PROCEDURE_RTH);
}

failsafePhase_e failsafePhase(void)
{
    return failsafeState.phase;
}

bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

bool failsafeIsActive(void)
{
    return failsafeState.active;
}

bool failsafeShouldApplyControlInput(void)
{
    return failsafeState.controlling;
}

bool failsafeRequiresAngleMode(void)
{
    return failsafeState.active &&
           failsafeState.controlling &&
           failsafeProcedureLogic[failsafeState.activeProcedure].forceAngleMode;
}

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return failsafeConfig()->failsafe_off_delay && (millis() > failsafeState.landingShouldBeFinishedAt);
}

static void failsafeSetActiveProcedure(failsafeProcedure_e procedure)
{
    failsafeState.activeProcedure = procedure;
}

static void failsafeActivate(failsafePhase_e newPhase)
{
    failsafeState.active = true;
    failsafeState.controlling = true;
    failsafeState.phase = newPhase;
    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
    failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;

#ifdef USE_FW_AUTOLAND
    posControl.fwLandState.landAborted = false;
#endif

    failsafeState.events++;
}

void failsafeUpdateRcCommandValues(void)
{
    if (!failsafeState.active) {
        for (int idx = 0; idx < 4; idx++) {
            failsafeState.lastGoodRcCommand[idx] = rcCommand[idx];
        }
    }
}

void failsafeApplyControlInput(void)
{
    // Apply channel values
    for (int idx = 0; idx < 4; idx++) {
        switch (failsafeProcedureLogic[failsafeState.activeProcedure].channelBehavior[idx]) {
            case FAILSAFE_CHANNEL_HOLD:
                rcCommand[idx] = failsafeState.lastGoodRcCommand[idx];
                break;

            case FAILSAFE_CHANNEL_NEUTRAL:
                switch (idx) {
                    case ROLL:
                    case PITCH:
                    case YAW:
                        rcCommand[idx] = 0;
                        break;

                    case THROTTLE:
                        rcCommand[idx] = feature(FEATURE_REVERSIBLE_MOTORS) ? PWM_RANGE_MIDDLE : getThrottleIdleValue();
                        break;
                }
                break;
        }
    }
}

bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
}

void failsafeOnRxSuspend(void)
{
    failsafeState.suspended = true;
}

bool failsafeIsSuspended(void)
{
    return failsafeState.suspended;
}

void failsafeOnRxResume(void)
{
    failsafeState.suspended = false;                                    // restart monitoring
    failsafeState.validRxDataReceivedAt = millis();                     // prevent RX link down trigger, restart rx link up
    failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;                     // do so while rx link is up
}

void failsafeOnValidDataReceived(void)
{
    failsafeState.validRxDataReceivedAt = millis();
    if ((failsafeState.validRxDataReceivedAt - failsafeState.validRxDataFailedAt) > failsafeState.rxDataRecoveryPeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
    }
}

void failsafeOnValidDataFailed(void)
{
    failsafeState.validRxDataFailedAt = millis();
    if ((failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt) > failsafeState.rxDataFailurePeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
    }
}

static bool failsafeCheckStickMotion(void)
{
    if (failsafeConfig()->failsafe_stick_motion_threshold > 0) {
        uint32_t totalRcDelta = 0;

        totalRcDelta += ABS(rxGetChannelValue(ROLL) - PWM_RANGE_MIDDLE);
        totalRcDelta += ABS(rxGetChannelValue(PITCH) - PWM_RANGE_MIDDLE);
        totalRcDelta += ABS(rxGetChannelValue(YAW) - PWM_RANGE_MIDDLE);

        return totalRcDelta >= failsafeConfig()->failsafe_stick_motion_threshold;
    }
    else {
        return true;
    }
}

static failsafeProcedure_e failsafeChooseFailsafeProcedure(void)
{
    if ((FLIGHT_MODE(NAV_WP_MODE) || isWaypointMissionRTHActive()) && failsafeConfig()->failsafe_mission_delay) {
        if (!failsafeState.wpModeDelayedFailsafeStart) {
            failsafeState.wpModeDelayedFailsafeStart = millis();
            return FAILSAFE_PROCEDURE_NONE;
        } else if ((millis() - failsafeState.wpModeDelayedFailsafeStart < (MILLIS_PER_SECOND * (uint16_t)failsafeConfig()->failsafe_mission_delay)) ||
                   failsafeConfig()->failsafe_mission_delay == -1) {
            return FAILSAFE_PROCEDURE_NONE;
        }
    }

    // Inhibit Failsafe if emergency landing triggered manually or if landing is detected
    if (posControl.flags.manualEmergLandActive || STATE(LANDING_DETECTED)) {
        return FAILSAFE_PROCEDURE_NONE;
    }

    // Craft is closer than minimum failsafe procedure distance (if set to non-zero)
    // GPS must also be working, and home position set
    if (failsafeConfig()->failsafe_min_distance > 0 &&
            ((sensors(SENSOR_GPS) && STATE(GPS_FIX))
#ifdef USE_GPS_FIX_ESTIMATION
                || STATE(GPS_ESTIMATED_FIX)
#endif
                ) && STATE(GPS_FIX_HOME)) {

        // get the distance to the original arming point
        uint32_t distance = calculateDistanceToDestination(&posControl.rthState.originalHomePosition);
        if (distance < failsafeConfig()->failsafe_min_distance) {
            // Use the alternate, minimum distance failsafe procedure instead
            return failsafeConfig()->failsafe_min_distance_procedure;
        }
    }

    return failsafeConfig()->failsafe_procedure;
}

#ifdef USE_GPS_FIX_ESTIMATION
bool checkGPSFixFailsafe(void)
{
    if (STATE(GPS_ESTIMATED_FIX) && (FLIGHT_MODE(NAV_WP_MODE) || isWaypointMissionRTHActive()) && (failsafeConfig()->failsafe_gps_fix_estimation_delay >= 0)) {
        if (!failsafeState.wpModeGPSFixEstimationDelayedFailsafeStart) {
            failsafeState.wpModeGPSFixEstimationDelayedFailsafeStart = millis();
        } else if ((millis() - failsafeState.wpModeGPSFixEstimationDelayedFailsafeStart) > (MILLIS_PER_SECOND * (uint16_t)MAX(failsafeConfig()->failsafe_gps_fix_estimation_delay,7))) {
            if ( !posControl.flags.forcedRTHActivated ) {
                failsafeSetActiveProcedure(FAILSAFE_PROCEDURE_RTH);
                failsafeActivate(FAILSAFE_RETURN_TO_HOME);
                activateForcedRTH();
                return true;
            }
        }
    } else {
        failsafeState.wpModeGPSFixEstimationDelayedFailsafeStart = 0;
    }
    return false;
}
#endif

/* ===========================================================================
 * Smart Anti-Jamming Failsafe
 * ===========================================================================
 * Layered on top of the native failsafe FSM. While SMART_FS_IDLE it only
 * watches is_wideband_jamming_active. Once engaged it owns the failsafe system
 * (failsafeUpdateState() skips the native phase switch) and drives the NAV FSM
 * exclusively through INAV's public external-event API:
 *
 *   SMART_FS_DR_RTH      -> activateForcedRTH(), trackback suppressed
 *   SMART_FS_DR_RETRACE  -> activateForcedRTH(), trackback allowed
 *   SMART_FS_LAND        -> activateForcedEmergLanding()
 *
 * Progress is read back through getStateOfForcedRTH() /
 * getStateOfForcedEmergLanding(), which inspect posControl.navState.
 * Hand-back to the native FSM is done by setting failsafeState.phase to
 * FAILSAFE_LANDED (disarm + re-arm lock) or FAILSAFE_RX_LOSS_RECOVERED
 * (pilot override) and returning SMART_FS_IDLE.
 *
 * Non-blocking, O(1) per call, no dynamic allocation.
 */

STATIC_ASSERT(SMART_FS_OVERRIDE_CHANNEL < MAX_SUPPORTED_RC_CHANNEL_COUNT, smart_fs_override_channel_out_of_range);
STATIC_ASSERT(SMART_FS_OVERRIDE_LOW_US < SMART_FS_OVERRIDE_HIGH_US, smart_fs_override_thresholds_invalid);

static smartFailsafeState_t smartFs;

bool failsafeSmartIsControlling(void)
{
    return smartFs.phase != SMART_FS_IDLE;
}

bool failsafeSmartPowerSaveActive(void)
{
    return smartFs.powerSaveActive;
}

bool failsafeSmartSuppressTrackback(void)
{
    // Direct return was chosen (user prefers RTH, or retrace path is unaffordable)
    return smartFs.phase == SMART_FS_DR_RTH;
}

const smartFailsafeState_t *getSmartFailsafeState(void)
{
    return &smartFs;
}

static void smartFsSetPhase(smartFailsafePhase_e newPhase, timeMs_t currentTimeMs)
{
    smartFs.phase = newPhase;
    smartFs.phaseEnteredMs = currentTimeMs;
}

/* ---- Power saving ------------------------------------------------------- */

static void smartFsEnterPowerSave(void)
{
    if (smartFs.powerSaveActive) {
        return;
    }

    // LED strip (ledStripUpdate), lights (lightsUpdate) and the VTX power
    // scheduler (vtxUpdate) poll failsafeSmartPowerSaveActive()
    smartFs.powerSaveActive = true;

#if defined(USE_VTX_CONTROL) && SMART_FS_POWER_SAVE_VTX_PIT
    smartFs.vtxPitForcedByUs = false;
    vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t pitOnOff = 0;
        const bool pitStateKnown = vtxCommonGetPitMode(vtxDevice, &pitOnOff);
        if (!pitStateKnown || !pitOnOff) {
            vtxCommonSetPitMode(vtxDevice, 1);      // Queued by the VTX driver, non-blocking
            smartFs.vtxPitForcedByUs = true;
        }
    }
#endif
}

static void smartFsExitPowerSave(void)
{
    if (!smartFs.powerSaveActive) {
        return;
    }

    smartFs.powerSaveActive = false;

#if defined(USE_VTX_CONTROL) && SMART_FS_POWER_SAVE_VTX_PIT
    vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice && smartFs.vtxPitForcedByUs) {
        vtxCommonSetPitMode(vtxDevice, 0);          // Restore video (also helps locating the craft)
    }
    smartFs.vtxPitForcedByUs = false;
#endif
}

/* ---- Sensors / user preference ------------------------------------------ */

static bool smartFsGpsIsDead(void)
{
    // STATE(GPS_FIX) is cleared while an *estimated* fix is used, so dead-reckoned fixes count as dead
    return !(sensors(SENSOR_GPS) && STATE(GPS_FIX));
}

static smartFailsafePreference_e smartFsReadUserPreference(void)
{
    if (failsafeConfig()->failsafe_procedure != FAILSAFE_PROCEDURE_RTH) {
        return SMART_FS_PREF_LAND;
    }

    if (navConfig()->general.flags.rth_trackback_mode != RTH_TRACKBACK_OFF) {
        return SMART_FS_PREF_RETRACE;
    }

    return SMART_FS_PREF_RTH;
}

/* ---- Battery model -------------------------------------------------------
 * required = (distance / nav_auto_speed + reserve) * I * margin
 * I = max(SMART_FS_ASSUMED_CURRENT_MA, measured current)
 * Remaining comes from getBatteryRemainingCapacity(), which already excludes
 * the user's battery_capacity_critical reserve.
 */

static uint32_t smartFsCapacityForTime(uint32_t flightTimeS, uint32_t currentMa)
{
    uint64_t mAh = ((uint64_t)currentMa * flightTimeS) / 3600U;
    mAh = (mAh * SMART_FS_SAFETY_MARGIN_PERCENT) / 100U;

    if (batteryMetersConfig()->capacity_unit == BAT_CAPACITY_UNIT_MWH) {
        // mWh = mAh * V ; getBatteryVoltage() is in 0.01V
        mAh = (mAh * getBatteryVoltage()) / 100U;
    }

    return (mAh > UINT32_MAX) ? UINT32_MAX : (uint32_t)mAh;
}

static void smartFsEvaluateBattery(timeMs_t currentTimeMs)
{
    smartFs.lastBatteryCheckMs = currentTimeMs;

    // If GPS came back, refresh the distance; otherwise keep the value latched at engage
    if (!smartFsGpsIsDead() && STATE(GPS_FIX_HOME)) {
        smartFs.homeDistanceCm = posControl.homeDistance;
    }

    const uint32_t speedCms = MAX((uint32_t)navConfig()->general.auto_speed, (uint32_t)SMART_FS_MIN_RTH_SPEED_CMS);
    const uint32_t rthTimeS = smartFs.homeDistanceCm / speedCms + SMART_FS_CLIMB_LAND_RESERVE_S;
    // Retrace upper bound: full trackback leg + direct leg home from its end
    const uint32_t retraceDistanceCm = smartFs.homeDistanceCm + (uint32_t)navConfig()->general.rth_trackback_distance * 100U;
    const uint32_t retraceTimeS = retraceDistanceCm / speedCms + SMART_FS_CLIMB_LAND_RESERVE_S;

    uint32_t currentMa = SMART_FS_ASSUMED_CURRENT_MA;
    if (isAmperageConfigured()) {
        const int32_t measuredMa = (int32_t)getAmperage() * 10;    // getAmperage() is in 0.01A
        if (measuredMa > (int32_t)currentMa) {
            currentMa = (uint32_t)measuredMa;
        }
    }

    smartFs.requiredRthCapacity     = smartFsCapacityForTime(rthTimeS, currentMa);
    smartFs.requiredRetraceCapacity = smartFsCapacityForTime(retraceTimeS, currentMa);

    smartFs.capacityKnown = isAmperageConfigured() &&
                            (currentBatteryProfile->capacity.value > 0) &&
                            batteryWasFullWhenPluggedIn();
    smartFs.remainingCapacity = smartFs.capacityKnown ? getBatteryRemainingCapacity() : 0;
}

static bool smartFsBatteryCritical(void)
{
    if (getBatteryState() == BATTERY_CRITICAL) {
        return true;
    }

    // Not enough usable capacity even for the shortest (direct) return
    return smartFs.capacityKnown && (smartFs.remainingCapacity < smartFs.requiredRthCapacity);
}

static bool smartFsBatterySufficientForRetrace(void)
{
    if (smartFs.capacityKnown) {
        return smartFs.remainingCapacity >= smartFs.requiredRetraceCapacity;
    }

    // No capacity data (no current sensor / capacity not set / pack not full at plug-in):
    // voltage-only fallback, only retrace on a healthy pack
    return getBatteryState() == BATTERY_OK;
}

/* ---- Decision tree ------------------------------------------------------ */

static smartFailsafePhase_e smartFsDecide(void)
{
    // 1. Critically low battery -> land where we are
    if (smartFsBatteryCritical()) {
        return SMART_FS_LAND;
    }

    // No home recorded -> nowhere to return to
    if (!STATE(GPS_FIX_HOME)) {
        return SMART_FS_LAND;
    }

    // GPS dead and IMU dead-reckoning already beyond its drift budget -> no blind return
    if (smartFsGpsIsDead() && deadReckoningDriftTooHigh()) {
        return SMART_FS_LAND;
    }

    // 2. User preferred RTH but GPS is dead -> dead-reckoning RTH
    if ((smartFs.preference == SMART_FS_PREF_RTH) && smartFsGpsIsDead()) {
        return SMART_FS_DR_RTH;
    }

    // 3. User preferred RETRACE and battery covers the longer path -> retrace
    if ((smartFs.preference == SMART_FS_PREF_RETRACE) && smartFsBatterySufficientForRetrace()) {
        return SMART_FS_DR_RETRACE;
    }

    // RTH with live GPS, or RETRACE unaffordable but direct return still is
    if ((smartFs.preference == SMART_FS_PREF_RTH) || (smartFs.preference == SMART_FS_PREF_RETRACE)) {
        return SMART_FS_DR_RTH;
    }

    // User preference LAND / DROP / NONE: under jamming, a controlled landing is the safe override
    return SMART_FS_LAND;
}

/* ---- Navigation interface ----------------------------------------------- */

static void smartFsCommandNavigation(smartFailsafePhase_e phase)
{
    switch (phase) {
        case SMART_FS_DR_RTH:
        case SMART_FS_DR_RETRACE:
            if (posControl.flags.forcedEmergLandingActivated) {
                abortForcedEmergLanding();
            }
            // Native FSM may already be retracing; restart RTH so the trackback suppression takes effect
            if ((phase == SMART_FS_DR_RTH) && posControl.flags.forcedRTHActivated && posControl.flags.rthTrackbackActive) {
                abortForcedRTH();
            }
            failsafeSetActiveProcedure(FAILSAFE_PROCEDURE_RTH);
            failsafeState.phase = FAILSAFE_RETURN_TO_HOME;
            if (!posControl.flags.forcedRTHActivated) {
                // NAV: -> NAV_STATE_RTH_INITIALIZE. Without a usable position estimate it waits
                // nav_position_timeout and then falls into NAV_STATE_EMERGENCY_LANDING_*.
                activateForcedRTH();
            }
            break;

        case SMART_FS_LAND:
            if (posControl.flags.forcedRTHActivated) {
                abortForcedRTH();
            }
            failsafeSetActiveProcedure(FAILSAFE_PROCEDURE_AUTO_LANDING);
            failsafeState.phase = FAILSAFE_LANDING;
            failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;
            if (!posControl.flags.forcedEmergLandingActivated) {
                // NAV: -> NAV_STATE_EMERGENCY_LANDING_INITIALIZE
                activateForcedEmergLanding();
            }
            break;

        default:
            break;
    }
}

/* ---- Manual override (AUX toggle after the jammer stops) ----------------- */

static void smartFsResetOverride(void)
{
    smartFs.overrideBaseline = SMART_FS_SWITCH_UNKNOWN;
    smartFs.overrideCandidate = SMART_FS_SWITCH_UNKNOWN;
    smartFs.overrideCandidateSinceMs = 0;
}

static smartFailsafeSwitchPos_e smartFsReadOverrideSwitch(void)
{
    if (SMART_FS_OVERRIDE_CHANNEL >= rxRuntimeConfig.channelCount) {
        return SMART_FS_SWITCH_UNKNOWN;
    }

    const int16_t value = rxGetChannelValue(SMART_FS_OVERRIDE_CHANNEL);

    if (value >= SMART_FS_OVERRIDE_HIGH_US) {
        return SMART_FS_SWITCH_HIGH;
    }
    if (value <= SMART_FS_OVERRIDE_LOW_US) {
        return SMART_FS_SWITCH_LOW;
    }
    return SMART_FS_SWITCH_UNKNOWN;
}

static bool smartFsManualOverrideRequested(timeMs_t currentTimeMs)
{
    // Only listen once the jammer is gone and the RC link is healthy again.
    // While the link is down AUX values are rxfail substitutes, not pilot input.
    if (is_wideband_jamming_active || !rxIsReceivingSignal() || !failsafeIsReceivingRxData() || IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        smartFsResetOverride();
        return false;
    }

    const smartFailsafeSwitchPos_e pos = smartFsReadOverrideSwitch();

    if (pos == SMART_FS_SWITCH_UNKNOWN) {
        smartFs.overrideCandidate = SMART_FS_SWITCH_UNKNOWN;
        return false;
    }

    // First trustworthy reading after link recovery becomes the reference position;
    // the pilot must actively flip the switch away from it.
    if (smartFs.overrideBaseline == SMART_FS_SWITCH_UNKNOWN) {
        smartFs.overrideBaseline = pos;
        smartFs.overrideCandidate = SMART_FS_SWITCH_UNKNOWN;
        return false;
    }

    if (pos == smartFs.overrideBaseline) {
        smartFs.overrideCandidate = SMART_FS_SWITCH_UNKNOWN;
        return false;
    }

    if (smartFs.overrideCandidate != pos) {
        smartFs.overrideCandidate = pos;
        smartFs.overrideCandidateSinceMs = currentTimeMs;
        return false;
    }

    return (timeMs_t)(currentTimeMs - smartFs.overrideCandidateSinceMs) >= SMART_FS_OVERRIDE_DEBOUNCE_MS;
}

/* ---- State machine ------------------------------------------------------ */

void executeSmartFailsafe(uint32_t currentTimeMs)
{
    const bool armed = ARMING_FLAG(ARMED);
    bool reprocessState;

    do {
        reprocessState = false;

        switch (smartFs.phase) {
            case SMART_FS_IDLE:
                if (is_wideband_jamming_active && armed && !STATE(LANDING_DETECTED) &&
                    (failsafeState.phase != FAILSAFE_LANDED) && (failsafeState.phase != FAILSAFE_RX_LOSS_MONITORING)) {
                    smartFsSetPhase(SMART_FS_ENGAGE, currentTimeMs);
                    reprocessState = true;
                }
                break;

            case SMART_FS_ENGAGE:
            {
                smartFs.engagements++;
                smartFs.preference = smartFsReadUserPreference();
                smartFs.homeDistanceCm = STATE(GPS_FIX_HOME) ? posControl.homeDistance : 0;
                smartFsResetOverride();

                // 1. Power saving, immediately
                smartFsEnterPowerSave();

                // Take ownership of the failsafe system (stick neutralisation, FAILSAFE_MODE, forced angle)
                if (!failsafeState.active) {
                    failsafeActivate(FAILSAFE_RX_LOSS_DETECTED);   // phase is overwritten by smartFsCommandNavigation()
                } else {
                    failsafeState.controlling = true;
                    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                }

                // 2. Battery check + 3. decision tree
                smartFsEvaluateBattery(currentTimeMs);
                const smartFailsafePhase_e decision = smartFsDecide();
                smartFsSetPhase(decision, currentTimeMs);
                smartFsCommandNavigation(decision);
                break;
            }

            case SMART_FS_DR_RTH:
            case SMART_FS_DR_RETRACE:
                if (smartFsManualOverrideRequested(currentTimeMs)) {
                    smartFsSetPhase(SMART_FS_MANUAL_OVERRIDE, currentTimeMs);
                    reprocessState = true;
                    break;
                }

                if (!armed) {
                    smartFsSetPhase(SMART_FS_LANDED, currentTimeMs);
                    reprocessState = true;
                    break;
                }

                // IMU drift watchdog: blind RTH has exceeded MAX_DEAD_RECKONING_TIME_MS / error budget
                if (smartFsGpsIsDead() && deadReckoningDriftTooHigh()) {
                    smartFsSetPhase(SMART_FS_LAND, currentTimeMs);
                    smartFsCommandNavigation(SMART_FS_LAND);
                    break;
                }

                // Battery keeps draining while we fly home - downgrade to LAND if it no longer suffices
                if ((timeMs_t)(currentTimeMs - smartFs.lastBatteryCheckMs) >= SMART_FS_BATTERY_RECHECK_MS) {
                    smartFsEvaluateBattery(currentTimeMs);
                    if (smartFsBatteryCritical()) {
                        smartFsSetPhase(SMART_FS_LAND, currentTimeMs);
                        smartFsCommandNavigation(SMART_FS_LAND);
                        break;
                    }
                }

                switch (getStateOfForcedRTH()) {
                    case RTH_IN_PROGRESS:
                        // NAV_STATE_RTH_* / NAV_STATE_EMERGENCY_LANDING_* (nav's own fallback)
                        break;

                    case RTH_HAS_LANDED:
                        smartFsSetPhase(SMART_FS_LANDED, currentTimeMs);
                        reprocessState = true;
                        break;

                    case RTH_IDLE:
                    default:
                        // NAV dropped/refused RTH - land in place
                        smartFsSetPhase(SMART_FS_LAND, currentTimeMs);
                        smartFsCommandNavigation(SMART_FS_LAND);
                        break;
                }
                break;

            case SMART_FS_LAND:
                if (smartFsManualOverrideRequested(currentTimeMs)) {
                    smartFsSetPhase(SMART_FS_MANUAL_OVERRIDE, currentTimeMs);
                    reprocessState = true;
                    break;
                }

                if (!armed) {
                    smartFsSetPhase(SMART_FS_LANDED, currentTimeMs);
                    reprocessState = true;
                    break;
                }

                switch (getStateOfForcedEmergLanding()) {
                    case EMERG_LAND_IN_PROGRESS:
                        if (failsafeShouldHaveCausedLandingByNow()) {
                            smartFsSetPhase(SMART_FS_LANDED, currentTimeMs);
                            reprocessState = true;
                        }
                        break;

                    case EMERG_LAND_HAS_LANDED:
                        smartFsSetPhase(SMART_FS_LANDED, currentTimeMs);
                        reprocessState = true;
                        break;

                    case EMERG_LAND_IDLE:
                    default:
                        // Same fallback as the native FSM: emergency landing impossible -> drop
                        abortForcedEmergLanding();
                        failsafeSetActiveProcedure(FAILSAFE_PROCEDURE_DROP_IT);
                        smartFsSetPhase(SMART_FS_LANDED, currentTimeMs);
                        reprocessState = true;
                        break;
                }
                break;

            case SMART_FS_LANDED:
                // Hand back to native FSM: FAILSAFE_LANDED disarms and locks re-arming
                smartFsExitPowerSave();
                failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS;
                failsafeState.phase = FAILSAFE_LANDED;
                smartFsSetPhase(SMART_FS_IDLE, currentTimeMs);
                break;

            case SMART_FS_MANUAL_OVERRIDE:
                // Jammer gone + pilot toggled AUX: abort nav, native FSM releases control
                if (posControl.flags.forcedRTHActivated) {
                    abortForcedRTH();
                }
                if (posControl.flags.forcedEmergLandingActivated) {
                    abortForcedEmergLanding();
                }
                smartFsExitPowerSave();
                failsafeSetActiveProcedure(failsafeConfig()->failsafe_procedure);
                failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;   // -> FAILSAFE_IDLE, clears FAILSAFE_MODE
                smartFsSetPhase(SMART_FS_IDLE, currentTimeMs);
                break;

            default:
                smartFsExitPowerSave();
                smartFsSetPhase(SMART_FS_IDLE, currentTimeMs);
                break;
        }
    } while (reprocessState);
}


void failsafeUpdateState(void)
{
    if (!failsafeIsMonitoring() || failsafeIsSuspended()) {
        return;
    }

    const bool receivingRxDataAndNotFailsafeMode = failsafeIsReceivingRxData() && !IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    const bool armed = ARMING_FLAG(ARMED);
    const bool sticksAreMoving = failsafeCheckStickMotion();
    beeperMode_e beeperMode = BEEPER_SILENCE;

    // Smart Anti-Jamming Failsafe has priority over the native procedure selection
    executeSmartFailsafe(millis());
    if (failsafeSmartIsControlling()) {
        if (armed) {
            beeper(BEEPER_RX_LOST_LANDING);
        }
        return;
    }

    // Beep RX lost only if we are not seeing data and we have been armed earlier
    if (!receivingRxDataAndNotFailsafeMode && ARMING_FLAG(WAS_EVER_ARMED)) {
        beeperMode = BEEPER_RX_LOST;
    }

    bool reprocessState;

    do {
        reprocessState = false;

        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:
                if (armed) {
                    // Track throttle command below minimum time
                    if (!throttleStickIsLow()) {
                        failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                    }

#ifdef USE_GPS_FIX_ESTIMATION
                    if ( checkGPSFixFailsafe() ) {
                        reprocessState = true;
                    } else
#endif
                    if (!receivingRxDataAndNotFailsafeMode) {
                        if ((failsafeConfig()->failsafe_throttle_low_delay && (millis() > failsafeState.throttleLowPeriod)) || STATE(NAV_MOTOR_STOP_OR_IDLE)) {
                            // JustDisarm: throttle was LOW for at least 'failsafe_throttle_low_delay' seconds or waiting for launch
                            // Don't disarm at all if `failsafe_throttle_low_delay` is set to zero
                            failsafeSetActiveProcedure(FAILSAFE_PROCEDURE_DROP_IT);
                            failsafeActivate(FAILSAFE_LANDED);  // skip auto-landing procedure
                            failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                            failsafeState.wpModeDelayedFailsafeStart = 0;
                        }
                        reprocessState = true;
                    }
                } else {
                    // When NOT armed, show rxLinkState of failsafe switch in GUI (failsafe mode)
                    if (!receivingRxDataAndNotFailsafeMode) {
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    } else {
                        DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    }
                    // Throttle low period expired (= low long enough for JustDisarm)
                    failsafeState.throttleLowPeriod = 0;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:
                if (receivingRxDataAndNotFailsafeMode) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                } else {
                    // Set active failsafe procedure
                    failsafeSetActiveProcedure(failsafeChooseFailsafeProcedure());

                    switch (failsafeState.activeProcedure) {
                        case FAILSAFE_PROCEDURE_AUTO_LANDING:
                            // Use Emergency Landing if Nav defined (otherwise stabilize and set Throttle to specified level).
                            failsafeActivate(FAILSAFE_LANDING);
                            activateForcedEmergLanding();
                            break;

                        case FAILSAFE_PROCEDURE_DROP_IT:
                            // Drop the craft
                            failsafeActivate(FAILSAFE_LANDED);      // skip auto-landing procedure
                            failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
                            break;

                        case FAILSAFE_PROCEDURE_RTH:
                            // Proceed to handling & monitoring RTH navigation
                            failsafeActivate(FAILSAFE_RETURN_TO_HOME);
                            activateForcedRTH();
                            break;
                        case FAILSAFE_PROCEDURE_NONE:
                        default:
                            // Do nothing procedure
                            failsafeActivate(FAILSAFE_RX_LOSS_IDLE);
                            break;
                    }
                }
                reprocessState = true;
                break;

            /* A very simple do-nothing failsafe procedure. The only thing it will do is monitor the receiver state and switch out of FAILSAFE condition */
            case FAILSAFE_RX_LOSS_IDLE:
                if (receivingRxDataAndNotFailsafeMode && sticksAreMoving) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                } else if (failsafeChooseFailsafeProcedure() != FAILSAFE_PROCEDURE_NONE) {  // trigger new failsafe procedure if changed
                    failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                    reprocessState = true;
                }
#ifdef USE_GPS_FIX_ESTIMATION
                else {
                    if ( checkGPSFixFailsafe() ) {
                        reprocessState = true;
                    }
                }
#endif

                break;

            case FAILSAFE_RETURN_TO_HOME:
                if (receivingRxDataAndNotFailsafeMode && sticksAreMoving) {
                    abortForcedRTH();
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                }
                else {
                    if (armed) {
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    }
                    bool rthLanded = false;
                    switch (getStateOfForcedRTH()) {
                        case RTH_IN_PROGRESS:
                            break;

                        case RTH_HAS_LANDED:
                            rthLanded = true;
                            break;

                        case RTH_IDLE:
                        default:
                            // This shouldn't happen. If RTH was somehow aborted during failsafe - fallback to FAILSAFE_LANDING procedure
                            abortForcedRTH();
                            failsafeSetActiveProcedure(FAILSAFE_PROCEDURE_AUTO_LANDING);
                            failsafeActivate(FAILSAFE_LANDING);
                            activateForcedEmergLanding();
                            reprocessState = true;
                            break;
                    }
                    if (rthLanded || !armed) {
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; // require 30 seconds of valid rxData
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;

            case FAILSAFE_LANDING:
                if (receivingRxDataAndNotFailsafeMode && sticksAreMoving) {
                    abortForcedEmergLanding();
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                } else {
                    if (armed) {
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    }
                    bool emergLanded = false;
                    switch (getStateOfForcedEmergLanding()) {
                        case EMERG_LAND_IN_PROGRESS:
                            break;

                        case EMERG_LAND_HAS_LANDED:
                            emergLanded = true;
                            break;

                        case EMERG_LAND_IDLE:
                        default:
                            // If emergency landing was somehow aborted during failsafe - fallback to FAILSAFE_PROCEDURE_DROP_IT
                            abortForcedEmergLanding();
                            failsafeSetActiveProcedure(FAILSAFE_PROCEDURE_DROP_IT);
                            failsafeActivate(FAILSAFE_LANDED);
                            reprocessState = true;
                            break;
                    }
                    if (emergLanded || failsafeShouldHaveCausedLandingByNow() || !armed) {
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; // require 30 seconds of valid rxData
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;

            case FAILSAFE_LANDED:
                ENABLE_ARMING_FLAG(ARMING_DISABLED_FAILSAFE_SYSTEM); // To prevent accidently rearming by an intermittent rx link
                disarm(DISARM_FAILSAFE);
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset; // set required period of valid rxData
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                failsafeState.controlling = false;  // Failsafe no longer in control of the machine - release control to pilot
                reprocessState = true;
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
                // Monitoring the rx link to allow rearming when it has become good for > `receivingRxDataPeriodPreset` time.
                if (receivingRxDataAndNotFailsafeMode) {
                    if (millis() > failsafeState.receivingRxDataPeriod) {
                        // rx link is good now, when arming via ARM switch, it must be OFF first
                        if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                            // XXX: Requirements for removing the ARMING_DISABLED_FAILSAFE_SYSTEM flag
                            // are tested by osd.c to show the user how to re-arm. If these
                            // requirements change, update osdArmingDisabledReasonMessage().
                            DISABLE_ARMING_FLAG(ARMING_DISABLED_FAILSAFE_SYSTEM);
                            failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                            reprocessState = true;
                        }
                    }
                } else {
                    failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                }
                break;

            case FAILSAFE_RX_LOSS_RECOVERED:
                // Entering IDLE with the requirement that throttle first must be at min_check for failsafe_throttle_low_delay period.
                // This is to prevent that JustDisarm is activated on the next iteration.
                // Because that would have the effect of shutting down failsafe handling on intermittent connections.
                failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = false;
                failsafeState.controlling = false;
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                reprocessState = true;
                break;

            default:
                break;
        }
    } while (reprocessState);

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}
