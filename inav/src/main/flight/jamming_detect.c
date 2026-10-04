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
 * Smart Anti-Jamming Failsafe - wide-band RF jamming detector.
 *
 * A wide-band jammer typically kills the 2.4 GHz RC link (AFHDS 2A) and the
 * L1 GNSS band at the same time. Either event alone is common (range, canopy,
 * multipath), but both collapsing together is a strong jamming signature.
 *
 * Non-blocking: called periodically (from processRx()), O(1), no allocation.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/time.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/gps.h"

#include "rx/rx.h"

#include "sensors/sensors.h"

#include "flight/jamming_detect.h"

bool is_wideband_jamming_active = false;

static jammingDetector_t jammingDetector;
static bool gpsHadValidFix = false;   // Only "losing" a fix counts, never "not yet having" one

static void jammingEnterState(jammingState_e newState, timeMs_t currentTimeMs)
{
    jammingDetector.state = newState;
    jammingDetector.stateEnteredMs = currentTimeMs;
}

/*
 * Evaluates RC link. Returns true if the link is degraded (below trigger).
 * *linkRecovered is set true only when the link is clearly healthy (above recover threshold).
 *
 * AFHDS 2A over iBUS carries no native RSSI, so RSSI comes from either
 * rssi_channel (FS-iA6B/iA10B RSSI on AUX) or ADC. Total packet loss (CRC
 * failures / no valid frames) is reported by rxIsReceivingSignal().
 */
static bool jammingEvaluateRfLink(bool *linkRecovered)
{
    // 100% packet loss / all frames failing CRC => worst case
    if (!rxIsReceivingSignal()) {
        jammingDetector.rssiPercent = 0;
        *linkRecovered = false;
        return true;
    }

    const uint8_t rssiPercent = (uint8_t)((getRSSI() * RSSI_VISIBLE_VALUE_MAX) / RSSI_MAX_VALUE);
    jammingDetector.rssiPercent = rssiPercent;

    // Receivers that report absolute power (CRSF, GHST, ...) - prefer dBm
    if (rxLinkStatistics.uplinkRSSI < 0) {
        const int16_t dBm = rxLinkStatistics.uplinkRSSI;
        *linkRecovered = (dBm > JAMMING_RSSI_RECOVER_DBM);
        return (dBm < JAMMING_RSSI_TRIGGER_DBM);
    }

    // Relative RSSI / LQ in percent (AFHDS 2A via RSSI channel or ADC)
    if (getRSSISource() != RSSI_SOURCE_NONE) {
        *linkRecovered = (rssiPercent >= JAMMING_RSSI_RECOVER_PERCENT);
        return (rssiPercent < JAMMING_RSSI_TRIGGER_PERCENT);
    }

    // No RSSI source configured: only total frame loss can be detected
    *linkRecovered = true;
    return false;
}

/*
 * Returns true if the GPS has LOST its 3D fix (it must have had one before).
 */
static bool jammingEvaluateGpsFixLost(void)
{
#ifdef USE_GPS
    if (!feature(FEATURE_GPS)) {
        return false;
    }

    const bool has3dFix = sensors(SENSOR_GPS) &&
                          isGPSHealthy() &&
                          STATE(GPS_FIX) &&
                          (gpsSol.fixType == GPS_FIX_3D);

    if (has3dFix) {
        gpsHadValidFix = true;
        return false;
    }

    return gpsHadValidFix;
#else
    return false;
#endif
}

void jammingDetectionInit(void)
{
    is_wideband_jamming_active = false;
    gpsHadValidFix = false;
    jammingDetector.state = JAMMING_STATE_IDLE;
    jammingDetector.stateEnteredMs = 0;
    jammingDetector.rssiPercent = 0;
    jammingDetector.rfDegraded = false;
    jammingDetector.gpsFixLost = false;
    jammingDetector.detectionCount = 0;
}

void updateJammingDetection(uint32_t currentTimeMs)
{
    bool rfRecovered = false;
    const bool rfDegraded = jammingEvaluateRfLink(&rfRecovered);
    const bool gpsFixLost = jammingEvaluateGpsFixLost();
    const bool jammingSignature = rfDegraded && gpsFixLost;

    jammingDetector.rfDegraded = rfDegraded;
    jammingDetector.gpsFixLost = gpsFixLost;

    // Unsigned subtraction is safe across millis() wrap-around
    const timeMs_t elapsedMs = (timeMs_t)(currentTimeMs - jammingDetector.stateEnteredMs);

    switch (jammingDetector.state) {
    case JAMMING_STATE_IDLE:
        if (jammingSignature) {
            jammingEnterState(JAMMING_STATE_SUSPECTED, currentTimeMs);
        }
        break;

    case JAMMING_STATE_SUSPECTED:
        if (!jammingSignature) {
            // Glitch - conditions did not persist
            jammingEnterState(JAMMING_STATE_IDLE, currentTimeMs);
        } else if (elapsedMs >= JAMMING_DETECT_DEBOUNCE_MS) {
            is_wideband_jamming_active = true;
            jammingDetector.detectionCount++;
            jammingEnterState(JAMMING_STATE_ACTIVE, currentTimeMs);
        }
        break;

    case JAMMING_STATE_ACTIVE:
        if (rfRecovered) {
            jammingEnterState(JAMMING_STATE_RECOVERING, currentTimeMs);
        }
        break;

    case JAMMING_STATE_RECOVERING:
        if (!rfRecovered) {
            // RF dropped again - restart recovery timing
            jammingEnterState(JAMMING_STATE_ACTIVE, currentTimeMs);
        } else if (elapsedMs > JAMMING_RECOVERY_DEBOUNCE_MS) {
            is_wideband_jamming_active = false;
            jammingEnterState(JAMMING_STATE_IDLE, currentTimeMs);
        }
        break;

    default:
        jammingDetectionInit();
        break;
    }
}

bool isWidebandJammingActive(void)
{
    return is_wideband_jamming_active;
}

const jammingDetector_t *getJammingDetectorState(void)
{
    return &jammingDetector;
}
