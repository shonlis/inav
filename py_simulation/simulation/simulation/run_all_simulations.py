"""Run all simulation and visualization scripts in sequence.

This master script executes all UAV failsafe simulation scenarios and generates
all figures for the engineering thesis. Close each figure window to proceed to
the next simulation.

Requirements: Python 3.9+, NumPy, Matplotlib.
"""

import sys
import os
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive windows
import matplotlib.pyplot as plt

# Enable interactive mode
plt.ion()


def run_script(script_path, script_name):
    """Load and execute a Python script as a module, then run its main block."""
    
    print(f"\n{'='*70}")
    print(f"Running: {script_name}")
    print(f"Path: {os.path.basename(script_path)}")
    print(f"{'='*70}\n")

    # Resolve to absolute path
    abs_path = os.path.abspath(script_path)
    if not os.path.exists(abs_path):
        print(f"ERROR: File not found at specified location")
        return False

    try:
        # Change to script directory so relative imports work
        original_dir = os.getcwd()
        os.chdir(os.path.dirname(abs_path))
        
        # Read the script content
        with open(abs_path, 'r', encoding='utf-8') as f:
            script_content = f.read()
        
        # Create a namespace dictionary with __name__ set to "__main__" so if __name__ == "__main__": blocks execute
        namespace = {
            '__name__': '__main__',
            '__file__': abs_path,
        }
        
        # Execute the script in this namespace
        exec(script_content, namespace)
        
        # Collect and save figures
        fig_nums = plt.get_fignums()
        if fig_nums:
            print(f"   Generated {len(fig_nums)} figure(s)")
            # Save figures to file
            output_dir = os.path.join(os.path.dirname(abs_path), 'output')
            os.makedirs(output_dir, exist_ok=True)
            for i, fig_num in enumerate(fig_nums):
                safe_name = script_name.replace(' ', '_').replace('(', '').replace(')', '').replace('/', '_')
                fig_file = os.path.join(output_dir, f"{safe_name}_{i}.png")
                try:
                    plt.figure(fig_num).savefig(fig_file, dpi=100, bbox_inches='tight')
                    print(f"   Saved: output/{os.path.basename(fig_file)}")
                except Exception as save_err:
                    print(f"   Could not save figure: {save_err}")
            
            # BLOCKING SHOW: Display figures and wait for user to close them
            print(f"\n   >>> Close all figure windows to continue to the next simulation <<<\n")
            plt.show(block=True)
        
        # Clear figures after display
        plt.close('all')
        
        os.chdir(original_dir)
        print(f"? {script_name} completed successfully.\n")
        return True
    except Exception as e:
        os.chdir(original_dir)
        print(f"? {script_name} failed with error:\n{e}\n")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Execute all simulation scripts in logical order."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    scripts = [
        (
            os.path.join(script_dir, "uav_failsafe_sim_final.py"),
            "Four Comprehensive Failsafe Scenarios (Telemetry & Trajectories)",
        ),
        (
            os.path.join(script_dir, "battery_capacity_failsafe_final.py"),
            "Battery Capacity Analysis (1.5 Ah vs 2.2 Ah LiPo)",
        ),
        (
            os.path.join(script_dir, "afhds_failsafe_timing.py"),
            "AFHDS 2A Millisecond-Resolution Response Time (260 Hz)",
        ),
        (
            os.path.join(script_dir, "imu_dead_reckoning_drift.py"),
            "IMU-Only Dead Reckoning Drift Analysis (30 seconds)",
        ),
        (
            os.path.join(script_dir, "wideband_jamming_spectrogram.py"),
            "Wideband RF Jamming Spectrogram (1.0-3.0 GHz)",
        ),
        (
            os.path.join(script_dir, "five_failsafe_scenarios.py"),
            "Five Simulated Flight Test Scenarios with Response Timing",
        ),
    ]

    print("\n" + "="*70)
    print("UAV SMART FAILSAFE SIMULATION SUITE")
    print("="*70)
    print(f"Running {len(scripts)} simulation scripts...\n")

    results = {}
    for script_path, script_name in scripts:
        results[script_name] = run_script(script_path, script_name)

    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    print(f"\nCompleted: {passed}/{total} scripts\n")

    for script_name, success in results.items():
        status = "? PASS" if success else "? FAIL"
        print(f"{status}: {script_name}")

    print("\n" + "="*70)
    print("All simulation figures are now displayed. Close each window to continue")
    print("or exit to end the run.")
    print("="*70 + "\n")

    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
