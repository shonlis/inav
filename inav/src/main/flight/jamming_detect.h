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

// ---- Tunables -------------------------------------------------------------
#define JAMMING_RSSI_TRIGGER_PERCENT    10      // RC RSSI/LQ below this [%] => RF degraded
#define JAMMING_RSSI_RECOVER_PERCENT    15      // RC RSSI/LQ must exceed this [%] to recover (hysteresis)
#define JAMMING_RSSI_TRIGGER_DBM        (-105)  // Used only when RX reports dBm (CRSF/GHST/...)
#define JAMMING_RSSI_RECOVER_DBM        (-100)
#define JAMMING_DETECT_DEBOUNCE_MS      200     // both conditions must hold this long to latch
#define JAMMING_RECOVERY_DEBOUNCE_MS    500     // RF must be good this long to clear

typedef enum {
    JAMMING_STATE_IDLE = 0,     // Link healthy, nothing suspicious
    JAMMING_STATE_SUSPECTED,    // RF low + GPS no 3D fix, debounce running
    JAMMING_STATE_ACTIVE,       // Jamming confirmed (flag latched)
    JAMMING_STATE_RECOVERING,   // Jamming latched, RF back above threshold, recovery debounce running
} jammingState_e;

typedef struct jammingDetector_s {
    jammingState_e state;
    timeMs_t       stateEnteredMs;   // Timestamp of entering current debounce state
    uint8_t        rssiPercent;      // Last evaluated RC RSSI/LQ [0..100]
    bool           rfDegraded;       // Last evaluated RF condition
    bool           gpsFixLost;       // Last evaluated GPS condition
    uint16_t       detectionCount;   // Number of confirmed jamming events since boot
} jammingDetector_t;

extern bool is_wideband_jamming_active;

void jammingDetectionInit(void);
void updateJammingDetection(uint32_t currentTimeMs);
bool isWidebandJammingActive(void);
const jammingDetector_t *getJammingDetectorState(void);
