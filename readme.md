# INAV – Smart Anti-Jamming Failsafe

הפרויקט מרחיב את קושחת **INAV 9.1.0** ומוסיף לה מנגנון חכם להתמודדות עם שיבוש רחב-סרט (wide-band jamming).
במצב כזה נופלים בו-זמנית ערוץ השליטה ב-2.4GHz (RC) וקליטת ה-GNSS בתדר L1.
המנגנון בנוי משלושה רכיבים:

1. **זיהוי שיבוש**: מזהה ירידה משולבת של איכות קישור ה-RC ואובדן קיבוע (fix) של ה-GPS.
2. **Failsafe חכם**: מחליט בזמן ריצה על הפעולה הנכונה – חזרה ישירה הביתה (RTH), חזרה במסלול שבו הכלי הגיע (Retrace) או נחיתה במקום. ההחלטה מתבססת על מצב הסוללה, על העדפת המשתמש ועל אמינות הניווט.
3. **ניווט עיוור (Dead Reckoning)**: מעריך את המיקום מה-IMU בלבד כשה-GPS משובש. רכיב watchdog מגביל את הסחיפה.

> הקישורים שבהמשך מפנים לשורות בקוד, והם יחסיים לשורש המאגר. אם הקוד משתנה, ייתכן שמספרי השורות ישתנו גם הם.

---

## 1. זיהוי שיבוש רחב-סרט – `jamming_detect`

| קטע קוד | תפקיד וחשיבות |
|---|---|
| [`jamming_detect.h` – פרמטרים וספים](src/main/flight/jamming_detect.h#L27-L33) | ספי RSSI (באחוזים וב-dBm) וזמני debounce. ספי ההפעלה וההתאוששות שונים זה מזה (היסטרזיס), וכך המצב לא "מהבהב" סביב סף אחד. |
| [`jammingEvaluateRfLink()`](src/main/flight/jamming_detect.c#L65-L101) | מעריכה את איכות קישור ה-RC. אם אין מסגרות תקינות, המצב נחשב לגרוע ביותר. כשהמקלט מדווח עוצמה מוחלטת (CRSF/GHST) נעשה שימוש ב-dBm, ובשאר המקרים ב-RSSI/LQ באחוזים (AFHDS 2A דרך ערוץ RSSI או ADC). הנוסחה: `rssi% = getRSSI() * 100 / RSSI_MAX_VALUE`. |
| [`jammingEvaluateGpsFixLost()`](src/main/flight/jamming_detect.c#L103-L127) | מחזירה "אמת" רק אם ה-GPS **איבד** קיבוע תלת-ממדי שהיה לו קודם. מצב שבו עדיין לא התקבל קיבוע (למשל מיד אחרי הדלקה) לא נחשב לשיבוש, וכך נמנעות התראות שווא. |
| [`updateJammingDetection()` – מכונת המצבים](src/main/flight/jamming_detect.c#L141-L192) | **אלגוריתם הזיהוי המרכזי.** המצבים הם `IDLE → SUSPECTED → ACTIVE → RECOVERING`. החתימה של שיבוש היא `rfDegraded && gpsFixLost`: כל אחד מהתנאים לבדו נפוץ (טווח, מבנים, multipath), אבל קריסה משותפת של שניהם מעידה בבירור על שיבוש. השיבוש "ננעל" רק אחרי `200ms` ומשתחרר רק אחרי `500ms` של קישור תקין. החישוב בטוח גם כש-`millis()` חוזר לאפס, כי החיסור הוא unsigned. |
| [קריאה מתוך לולאת ה-RX](src/main/fc/fc_core.c#L651-L655) | נקודת השילוב בקושחה. הזיהוי רץ **לפני** `failsafeUpdateState()`, ולכן ה-failsafe תמיד עובד עם הדגל `is_wideband_jamming_active` המעודכן. |

---

## 2. Failsafe חכם נגד שיבוש – `failsafe.c`

| קטע קוד | תפקיד וחשיבות |
|---|---|
| [`failsafe.h` – פרמטרים, מצבים ומבני נתונים](src/main/flight/failsafe.h#L186-L261) | הגדרת מכונת המצבים `smartFailsafePhase_e` (`IDLE / ENGAGE / DR_RTH / DR_RETRACE / LAND / LANDED / MANUAL_OVERRIDE`) ופרמטרי מודל הסוללה: זרם מוערך, מקדם ביטחון ועתודת זמן לטיפוס ולנחיתה. |
| [חיסכון בהספק – `smartFsEnterPowerSave()` / `smartFsExitPowerSave()`](src/main/flight/failsafe.c#L459-L500) | בזמן שיבוש המשדר (VTX) עובר ל-pit mode ונורות ה-LED כבות, כדי לחסוך באנרגיה לטובת החזרה. בסיום, הווידאו מוחזר רק אם המנגנון הוא זה שכיבה אותו, וכך קל יותר למצוא את הכלי. |
| [`smartFsReadUserPreference()`](src/main/flight/failsafe.c#L510-L521) | מתרגמת את הגדרות המשתמש הקיימות (`failsafe_procedure`, `nav_rth_trackback_mode`) להעדפה: LAND, RTH או RETRACE. |
| [**מודל הסוללה** – `smartFsCapacityForTime()` / `smartFsEvaluateBattery()`](src/main/flight/failsafe.c#L523-L573) | **הנוסחה המרכזית:**<br>`t_RTH = d_home / v_auto + t_reserve`<br>`t_RETRACE = (d_home + d_trackback) / v_auto + t_reserve`<br>`I = max(I_assumed, I_measured)`<br>`C_req[mAh] = I · t / 3600 · margin` (ב-mWh מכפילים גם במתח).<br>ערך המרחק מהבית נשמר ברגע הכניסה למצב, כי לאחר מכן ה-GPS כבר לא אמין. |
| [`smartFsBatteryCritical()` / `smartFsBatterySufficientForRetrace()`](src/main/flight/failsafe.c#L575-L594) | משוות את הקיבולת שנותרה לקיבולת הנדרשת. כשאין נתוני זרם או קיבולת, הן מסתמכות על מתח הסוללה בלבד. |
| [**עץ ההחלטות** – `smartFsDecide()`](src/main/flight/failsafe.c#L596-L632) | **לב ה-failsafe החכם.** סדר העדיפויות: (1) סוללה קריטית, אין נקודת בית או ש-DR כבר סחף ← נחיתה; (2) המשתמש העדיף RTH וה-GPS מת ← RTH עיוור; (3) המשתמש העדיף Retrace והסוללה מספיקה ← Retrace; אחרת ← RTH ישיר. אם ההעדפה היא LAND/DROP, הבחירה היא תמיד נחיתה מבוקרת. |
| [`smartFsCommandNavigation()`](src/main/flight/failsafe.c#L634-L673) | מפעילה את מכונת המצבים של הניווט ב-INAV **רק דרך ה-API הציבורי** (`activateForcedRTH`, `activateForcedEmergLanding`, `abort*`), בלי לשנות את הליבה. |
| [עקיפה ידנית – `smartFsManualOverrideRequested()`](src/main/flight/failsafe.c#L675-L737) | אחרי שהשיבוש מסתיים, הטייס יכול לקבל בחזרה את השליטה בהעברת מתג AUX4. הקוד שומר את מצב המתג הראשון כ"בסיס", מפעיל היסטרזיס (1300/1700µs) ו-debounce של 150ms, ומתעלם מערכי rxfail בזמן שהקישור עדיין מנותק. |
| [**מכונת המצבים** – `executeSmartFailsafe()`](src/main/flight/failsafe.c#L739-L899) | מנהלת את כל מחזור החיים: כניסה, ביצוע וניטור (watchdog לסחיפת DR, בדיקה חוזרת של הסוללה כל שנייה ומעבר לנחיתה בעת הצורך), ולבסוף החזרת השליטה ל-FSM המקורי (`FAILSAFE_LANDED` או `FAILSAFE_RX_LOSS_RECOVERED`). הקוד לא חוסם, כל קריאה היא O(1) ואין הקצאת זיכרון דינמית. |
| [העדיפות על ה-failsafe המקורי](src/main/flight/failsafe.c#L913-L920) | `failsafeUpdateState()` מריצה קודם את ה-failsafe החכם. כשהוא שולט, הבחירה המקורית של INAV מדולגת. |
| [ביטול ה-Trackback](src/main/navigation/rth_trackback.c#L42-L51) | אם הוחלט על חזרה ישירה (כי ה-Retrace יקר מדי או לא הועדף), מנגנון ה-trackback של INAV לא מופעל. |
| [VTX](src/main/io/vtx.c#L181-L182) · [LED Strip](src/main/io/ledstrip.c#L955) · [Lights](src/main/io/lights.c#L62) | נקודות השילוב של מצב החיסכון בהספק במודולים הקיימים. |

---

## 3. ניווט עיוור מה-IMU – `dead_reckoning`

| קטע קוד | תפקיד וחשיבות |
|---|---|
| [`dead_reckoning.h` – פרמטרים ומודל השגיאה](src/main/navigation/dead_reckoning.h#L29-L49) | מגבלות ה-watchdog (`20s` לכל היותר בלי GPS, שגיאה של עד `50m`), קבוע הזמן של הדליפה `τ=5s`, deadband, מגבלות מהירות פיזיקליות, ודגל ניסיוני לשליחת המיקום לבקר הניווט. |
| [Soft deadband – `drSoftDeadband()` / `drSoftDeadbandXY()`](src/main/navigation/dead_reckoning.c#L83-L107) | מסננת שאריות bias ורעש בתאוצה, בלי קפיצה בקצה התחום. בגרסה הרדיאלית (XY) כיוון הווקטור נשמר והיחס זהה בכל הכיוונים. |
| [**האינטגרטור** – `drIntegrate()`](src/main/navigation/dead_reckoning.c#L126-L165) | **אלגוריתם האינטגרציה:**<br>`v[k+1] = v[k] / (1 + dt/τ) + a'·dt` (אינטגרטור דולף ≡ HPF מסדר ראשון, Backward-Euler יציב לכל `dt`)<br>`p[k+1] = p[k] + ½·(v[k] + v[k+1])·dt` (טרפז).<br>היתרון: bias קבוע `b` גורם לשגיאת מהירות חסומה `b·τ`, במקום שגיאה שגדלה כ-`b·t`. לכן שגיאת המיקום גדלה ליניארית ולא ריבועית. בנוסף יש הגבלת מהירות פיזיקלית, והתעלמות מדגימות רוויות (clipping). |
| [**Watchdog הסחיפה** – `drUpdateWatchdog()`](src/main/navigation/dead_reckoning.c#L167-L194) | **מודל השגיאה:** `err(t) = eph₀ + V_err·t + ½·A_err·t²`. אם כיוון הטיסה (heading) כבר לא אמין, לדוגמה ב-MPU6500 שאין בו מגנטומטר, `A_err` גדל. אם עובר הזמן המרבי או שהשגיאה המוערכת חורגת מהסף, הדגל `drift_too_high` ננעל, וה-failsafe עובר לנחיתה. |
| [`updateDeadReckoningPosition()` – מעבר בין מעקב ל-DR](src/main/navigation/dead_reckoning.c#L214-L259) | כשה-GPS אמין, המודול עוקב אחרי ההערכה המשולבת (seed). כשה-GPS נופל, הוא עובר לאינטגרציה עצמאית. אמון ב-GPS חוזר רק אחרי `2s` רצופות של קיבוע תקין, וכך דגימה תקינה בודדת בזמן שיבוש לא מספיקה. |
| [שילוב במעריך המיקום](src/main/navigation/navigation_pos_estimator.c#L936-L940) · [פרסום לניווט](src/main/navigation/navigation_pos_estimator.c#L826-L831) | המודול רץ בקצב לולאת המעריך. כשהאפשרות פעילה, המיקום נשלח לניווט ברמת `USABLE` (ולא `TRUSTED`), ורק כל עוד ה-failsafe החכם שולט וגבול הסחיפה לא נחצה. |

---

## זרימה כללית

```
processRx() ──► updateJammingDetection()  ──► is_wideband_jamming_active
                                                     │
failsafeUpdateState() ──► executeSmartFailsafe() ◄───┘
                               │  ENGAGE: power-save → battery model → smartFsDecide()
                               ├─► DR_RTH / DR_RETRACE ─► activateForcedRTH()
                               └─► LAND               ─► activateForcedEmergLanding()
                                         ▲
updatePositionEstimator() ──► updateDeadReckoningPosition() ── drift_too_high (watchdog)
```

## בנייה

הבנייה זהה לבנייה הרגילה של INAV (CMake). הקבצים החדשים רשומים ב-[`src/main/CMakeLists.txt`](src/main/CMakeLists.txt#L320-L321) ([jamming_detect](src/main/CMakeLists.txt#L320-L321), [dead_reckoning](src/main/CMakeLists.txt#L579-L580)).
מידע נוסף על הבנייה נמצא ב-[`docs/development`](docs/development).


# UAV Smart Failsafe Simulation Suite - User Guide

## Overview
This simulation suite generates all thesis figures for the UAV Smart Failsafe system. It runs 6 comprehensive simulations, generating 12 figures total.

## How to Run

### From Command Line (Recommended)
```powershell
...\py_simulation\simulation\simulation"
python run_all_simulations.py
```

### From Python IDE (VS Code, PyCharm, etc.)
Simply run the `run_all_simulations.py` file directly.

## What to Expect

The simulation suite will:

1. **Run 6 simulation scripts sequentially:**
   - Four Comprehensive Failsafe Scenarios (5 figures)
   - Battery Capacity Analysis (2 figures)
   - AFHDS 2A Millisecond-Resolution Response Time (1 figure)
   - IMU-Only Dead Reckoning Drift Analysis (1 figure)
   - Wideband RF Jamming Spectrogram (1 figure)
   - Five Simulated Flight Test Scenarios (2 figures)

2. **For each simulation:**
   - Figures will OPEN in matplotlib windows
   - A message will display: "Close all figure windows to continue"
   - **Keep the windows open as long as you want to study the figures**
   - When you close ALL windows, the next simulation will automatically start

3. **All figures are saved to:**
   - `output/` subdirectory with descriptive PNG filenames
   - Figures are saved at 100 DPI for thesis quality

4. **Final summary:**
   - Displays pass/fail status for all 6 scripts
   - Shows total completion count (6/6 expected)

## Figure Output Files

All generated figures are saved in the `output/` directory:
- `Four_Comprehensive_Failsafe_Scenarios_Telemetry_&_Trajectories_*.png` (5 files)
- `Battery_Capacity_Analysis_1.5_Ah_vs_2.2_Ah_LiPo_*.png` (2 files)
- `AFHDS_2A_Millisecond-Resolution_Response_Time_260_Hz_*.png` (1 file)
- `IMU-Only_Dead_Reckoning_Drift_Analysis_30_seconds_*.png` (1 file)
- `Wideband_RF_Jamming_Spectrogram_1.0-3.0_GHz_*.png` (1 file)
- `Five_Simulated_Flight_Test_Scenarios_with_Response_Timing_*.png` (2 files)

## Key Code Sections (Algorithms & Formulas)

| Code section | Description & importance |
|---|---|
| [`DecisionEngine.choose`](uav_failsafe_sim_final.py#L131) | **Core failsafe decision algorithm.** Combines user preference (LAND / RTH / RETRACE), GPS availability and remaining battery to select `land`, `rth_gps`, `rth_dr`, `retrace_gps` or `retrace_dr`. Overrides to LAND when the battery is insufficient – the heart of the "smart failsafe". |
| [`DecisionEngine.est_rth`](uav_failsafe_sim_final.py#L112) | **RTH cost estimate:** `t = d_horiz / v_fs + h / v_descend`, `battery = t_h·drain_fs + t_d·drain_desc`. Provides the time/energy input for the decision. |
| [`DecisionEngine.est_retrace`](uav_failsafe_sim_final.py#L120) | **Retrace cost estimate:** sums waypoint segment lengths `Σ‖wp[i+1] − wp[i]‖` and converts to time and battery. Enables comparing retrace vs. direct RTH. |
| [`feasible`](uav_failsafe_sim_final.py#L149) | **Feasibility rule:** `battery − required ≥ safety_margin`. Guarantees a reserve on landing. |
| [`UAV.step`](uav_failsafe_sim_final.py#L251) | **State machine:** detects jamming when `RSSI < sensitivity`, activates power-save, invokes the decision engine and executes the chosen action each time step. |
| [`UAV._dead_reck_step`](uav_failsafe_sim_final.py#L419) | **Dead reckoning with drift:** integrates acceleration to velocity/position and adds a random-walk drift (`σ·√dt`), modeling navigation without GPS. |
| [`UAV._move_towards`](uav_failsafe_sim_final.py#L434) | **Guidance step:** moves toward a target at a fixed speed on a unit direction vector, with controlled descent rate. |
| [`mah_consumed`](battery_capacity_failsafe_final.py#L62) | **Energy formula:** `mAh = I[A] · t[s] · 1000 / 3600`. Basis for all capacity calculations. |
| [`theoretical_flight_time_minutes`](battery_capacity_failsafe_final.py#L67) | **Flight time:** `t[min] = (C[mAh]/1000) / I[A] · 60`. Compares 1.5 Ah vs. 2.2 Ah packs. |
| [`estimate_rth_capacity_mah`](battery_capacity_failsafe_final.py#L72) | **RTH capacity estimate** (horizontal return + descent at 15 A). |
| [`select_failsafe`](battery_capacity_failsafe_final.py#L81) | **Capacity-based selection:** RTH only if `remaining ≥ RTH_required + reserve`, otherwise LAND. |
| [`integrate_noisy_imu`](imu_dead_reckoning_drift.py#L52) | **IMU double integration:** `v[k] = v[k−1] + a_meas·dt`, `p[k] = p[k−1] + v[k]·dt`, with `a_meas = a_true + bias + noise`. Shows why IMU-only navigation drifts. |
| [`theoretical_bias_error`](imu_dead_reckoning_drift.py#L81) | **Bias error formula:** `e(t) = ½·‖b‖·t²`. Quadratic error growth – justifies limiting DR duration. |
| [`generate_timeline`](afhds_failsafe_timing.py#L38) | **AFHDS 2A timing (260 Hz):** builds the millisecond timeline from link loss to failsafe activation – demonstrates response time. |
| [`add_narrowband_signal`](wideband_jamming_spectrogram.py#L41) | **Spectrogram model:** injects narrowband signals into a 1–3 GHz spectrum to visualize wideband jamming. |
| [`FailsafeScenario.simulate`](five_failsafe_scenarios.py#L78) | **Flight-test scenarios:** simulates five scenarios and measures response time per scenario. |
| [`run_script` / `main`](run_all_simulations.py#L20) | **Runner:** executes all simulations sequentially and reports pass/fail. |

For detailed explanations see [`algorithm_explanations.md`](algorithm_explanations.md) and [`complete_algorithm_reference.md`](complete_algorithm_reference.md).

## Behavior Details

### Figure Display Mode
- **Blocking mode enabled**: Each simulation's figures stay open until you close them
- **Interactive windows**: You can pan, zoom, and inspect each figure
- **PNG backup**: All figures are automatically saved, even if windows are closed

### Order of Simulations
Figures appear in thesis-recommended order. Each simulation runs independently with complete telemetry.

### Requirements
- Python 3.9+ 
- NumPy
- Matplotlib (with TkAgg backend support)

## Troubleshooting

### Figures not appearing?
- Check that you're running from the correct directory
- Verify matplotlib has display capabilities
- Check if figures opened behind other windows
- Even if not displayed, PNG files will be saved in `output/`

### Script fails?
- Check console output for error messages
- Verify all simulation scripts exist in the same directory
- Ensure numpy and matplotlib are installed

### Want to re-run?
- Simply run the script again
- Existing PNG files in `output/` will be overwritten

## Integration with Thesis
Export the PNG files from `output/` directly to your thesis document. All figures have been generated with publication-quality formatting.
