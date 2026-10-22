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
