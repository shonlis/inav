"""Compare 3S LiPo 1500 mAh and 2200 mAh packs in an RF-jam failsafe simulation.

Run with Python, NumPy, and Matplotlib installed. The model uses a nominal 11.1 V
pack and a constant 15 A flight-current approximation; results are estimates, not
battery-performance guarantees. It creates a discharge/decision figure and a
separated 1x2 trajectory figure.
"""

import numpy as np
import matplotlib.pyplot as plt


# -----------------------------------------------------------------------------
# Simulation and vehicle parameters
# -----------------------------------------------------------------------------
SIM_SECONDS = 600.0
DT = 1.0
TIME = np.arange(0.0, SIM_SECONDS + DT, DT)

PACK_VOLTAGE_V = 11.1
FLIGHT_CURRENT_A = 15.0
BATTERY_CAPACITIES_MAH = {"1.5 Ah": 1500.0, "2.2 Ah": 2200.0}

CRUISE_SPEED_MPS = 4.0
RTH_SPEED_MPS = 3.0
START_ALTITUDE_M = 15.0
DESCENT_RATE_MPS = 0.6
JAM_TIME_S = 200.0
JAM_INDEX = int(JAM_TIME_S / DT)
BATTERY_RESERVE_FRACTION = 0.05

RSSI_NORMAL_DBM = -40.0
RSSI_JAMMED_DBM = -112.0
RSSI_THRESHOLD_DBM = -105.0
RANDOM_SEED = 7


# -----------------------------------------------------------------------------
# Sensor and path generation
# -----------------------------------------------------------------------------
def generate_true_path(time_s):
    """Return a simple curved route from home, with constant altitude."""
    x_m = CRUISE_SPEED_MPS * time_s
    y_m = 6.0 * np.sin(0.05 * time_s)
    z_m = np.full_like(time_s, START_ALTITUDE_M, dtype=float)
    return np.column_stack((x_m, y_m, z_m))


def generate_rssi(time_s, jam_index):
    """Generate strong pre-jam RSSI and a sharp drop below the detection threshold."""
    rng = np.random.default_rng(RANDOM_SEED)
    signal_dbm = RSSI_NORMAL_DBM + rng.normal(0.0, 0.8, size=len(time_s))
    signal_dbm[jam_index:] = RSSI_JAMMED_DBM + rng.normal(
        0.0, 0.8, size=len(time_s) - jam_index
    )
    return signal_dbm


# -----------------------------------------------------------------------------
# Battery calculations and failsafe choice
# -----------------------------------------------------------------------------
def mah_consumed(current_a, duration_s):
    """Convert current draw over time to consumed capacity in mAh."""
    return current_a * duration_s * 1000.0 / 3600.0


def theoretical_flight_time_minutes(capacity_mah, current_a=FLIGHT_CURRENT_A):
    """Ideal full-capacity flight time; excludes reserve and real-world losses."""
    return (capacity_mah / 1000.0) / current_a * 60.0


def estimate_rth_capacity_mah(start_position):
    """Estimate RTH energy at 15 A: horizontal return followed by vertical descent."""
    home_xy = np.zeros(2)
    horizontal_distance_m = np.linalg.norm(start_position[:2] - home_xy)
    horizontal_time_s = horizontal_distance_m / RTH_SPEED_MPS
    descent_time_s = max(0.0, start_position[2] / DESCENT_RATE_MPS)
    return mah_consumed(FLIGHT_CURRENT_A, horizontal_time_s + descent_time_s)


def select_failsafe(capacity_mah, remaining_mah, rth_required_mah):
    """Choose RTH if capacity covers estimated RTH energy plus a pack reserve."""
    reserve_mah = capacity_mah * BATTERY_RESERVE_FRACTION
    if remaining_mah >= rth_required_mah + reserve_mah:
        return "RTH (dead reckoning)", rth_required_mah + reserve_mah
    return "LAND (insufficient RTH capacity)", rth_required_mah + reserve_mah


# -----------------------------------------------------------------------------
# Scenario simulation
# -----------------------------------------------------------------------------
def simulate_battery_case(name, capacity_mah, true_path, rssi_dbm):
    """Run one capacity case and return battery, decision, and actual XY trajectory."""
    jam_position = true_path[JAM_INDEX].copy()
    rth_required_mah = estimate_rth_capacity_mah(jam_position)
    consumed_before_jam_mah = mah_consumed(FLIGHT_CURRENT_A, JAM_TIME_S)
    remaining_at_jam_mah = max(0.0, capacity_mah - consumed_before_jam_mah)
    decision, required_with_reserve_mah = select_failsafe(
        capacity_mah, remaining_at_jam_mah, rth_required_mah
    )

    positions = np.empty_like(true_path)
    positions[: JAM_INDEX + 1] = true_path[: JAM_INDEX + 1]
    positions[JAM_INDEX] = jam_position

    # Carry remaining capacity forward using flight current until the drone lands.
    remaining_mah = np.empty(len(TIME), dtype=float)
    remaining_mah[: JAM_INDEX + 1] = np.maximum(
        0.0,
        capacity_mah
        - mah_consumed(FLIGHT_CURRENT_A, TIME[: JAM_INDEX + 1]),
    )
    position = jam_position.copy()
    estimate = jam_position.copy()
    rng = np.random.default_rng(RANDOM_SEED + int(capacity_mah))
    landed = False

    # Make the dead-reckoning estimate drift gradually after GPS loss. The controller
    # follows the estimated home vector, while positions record the simulated vehicle.
    for k in range(JAM_INDEX + 1, len(TIME)):
        if landed:
            positions[k] = position
            remaining_mah[k] = remaining_mah[k - 1]
            continue

        if decision.startswith("LAND"):
            position[2] = max(0.0, position[2] - DESCENT_RATE_MPS * DT)
            estimate[2] = position[2]
            landed = position[2] <= 0.0
        else:
            to_home = -estimate[:2]
            estimated_distance = np.linalg.norm(to_home)
            if estimated_distance > RTH_SPEED_MPS * DT:
                commanded_step = to_home / estimated_distance * RTH_SPEED_MPS * DT
                position[:2] += commanded_step
                # IMU/DR position error accumulates as a small random walk.
                estimate[:2] += commanded_step + rng.normal(0.0, 0.025, size=2)
            else:
                position[:2] = 0.0
                estimate[:2] = 0.0
                position[2] = max(0.0, position[2] - DESCENT_RATE_MPS * DT)
                estimate[2] = position[2]
                landed = position[2] <= 0.0

        positions[k] = position
        remaining_mah[k] = max(
            0.0, remaining_mah[k - 1] - mah_consumed(FLIGHT_CURRENT_A, DT)
        )

    battery_percent = 100.0 * remaining_mah / capacity_mah
    return {
        "name": name,
        "capacity_mah": capacity_mah,
        "rssi_dbm": rssi_dbm,
        "positions": positions,
        "remaining_mah": remaining_mah,
        "battery_percent": battery_percent,
        "jam_position": jam_position,
        "remaining_at_jam_mah": remaining_at_jam_mah,
        "rth_required_mah": rth_required_mah,
        "required_with_reserve_mah": required_with_reserve_mah,
        "decision": decision,
    }


# -----------------------------------------------------------------------------
# Figures
# -----------------------------------------------------------------------------
def plot_battery_comparison(results):
    """Plot battery percentage, each pack's normalized RTH threshold, and jam time."""
    fig, ax = plt.subplots(figsize=(11, 6))
    colors = {"1.5 Ah": "tab:blue", "2.2 Ah": "tab:orange"}

    for result in results:
        color = colors[result["name"]]
        ax.plot(
            TIME,
            result["battery_percent"],
            color=color,
            linewidth=2.2,
            label=(
                f"{result['name']} remaining "
                f"(ideal flight time {theoretical_flight_time_minutes(result['capacity_mah']):.1f} min)"
            ),
        )
        threshold_percent = (
            result["required_with_reserve_mah"] / result["capacity_mah"] * 100.0
        )
        ax.axhline(
            threshold_percent,
            color=color,
            linestyle="--",
            alpha=0.8,
            label=f"{result['name']} RTH + 5% reserve threshold ({threshold_percent:.1f}%)",
        )
        ax.scatter(
            [JAM_TIME_S],
            [100.0 * result["remaining_at_jam_mah"] / result["capacity_mah"]],
            color=color,
            edgecolor="black",
            zorder=4,
        )

    ax.axvline(JAM_TIME_S, color="red", linestyle=":", linewidth=2, label="Jamming / GPS loss")
    ax.set(
        title="Battery Discharge and Dead-Reckoning RTH Feasibility",
        xlabel="Time (s)",
        ylabel="Remaining battery capacity (%)",
        xlim=(0.0, SIM_SECONDS),
        ylim=(0.0, 105.0),
    )
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", fontsize=8)

    summary = "\n".join(
        f"{r['name']}: {r['decision']} | jam remaining "
        f"{r['remaining_at_jam_mah']:.0f} mAh / "
        f"RTH+reserve {r['required_with_reserve_mah']:.0f} mAh"
        for r in results
    )
    ax.text(
        0.02,
        0.04,
        summary,
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.9),
    )
    fig.tight_layout()
    return fig


def plot_trajectory_comparison(results):
    """Plot each pack's flight and failsafe route in a separate subplot."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), sharex=False, sharey=False)
    colors = {"1.5 Ah": "tab:blue", "2.2 Ah": "tab:orange"}

    for ax, result in zip(axes, results):
        path = result["positions"]
        jam_xy = result["jam_position"][:2]
        color = colors[result["name"]]

        ax.plot(
            path[: JAM_INDEX + 1, 0],
            path[: JAM_INDEX + 1, 1],
            color="black",
            linewidth=1.8,
            label="True flight path before jam",
        )
        ax.plot(
            path[JAM_INDEX:, 0],
            path[JAM_INDEX:, 1],
            color=color,
            linewidth=2.2,
            label=f"Executed: {result['decision'].split(' (')[0]}",
        )
        ax.scatter(0.0, 0.0, color="green", marker="*", s=120, label="Home")
        ax.scatter(
            jam_xy[0], jam_xy[1], color="red", marker="x", s=90, linewidths=2.5, label="Jam / GPS loss"
        )
        ax.scatter(
            path[-1, 0], path[-1, 1], color=color, marker="o", s=55, label="Final position"
        )

        decision_text = (
            f"Pack: {result['name']} ({result['capacity_mah']:.0f} mAh, 11.1 V)\n"
            f"Jam: RSSI {RSSI_JAMMED_DBM:.0f} dBm; GPS lost\n"
            f"At jam: {result['remaining_at_jam_mah']:.0f} mAh remaining\n"
            f"Estimated RTH + reserve: {result['required_with_reserve_mah']:.0f} mAh\n"
            f"Final decision: {result['decision']}"
        )
        ax.text(
            0.03,
            0.97,
            decision_text,
            transform=ax.transAxes,
            va="top",
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.9),
        )
        ax.set_title(f"{result['name']} battery")
        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.axis("equal")
        ax.grid(True, alpha=0.35)
        ax.legend(loc="lower left", fontsize=8)

    fig.suptitle("Separated UAV Trajectories: 1.5 Ah vs 2.2 Ah", fontsize=14)
    fig.tight_layout()
    return fig


def print_capacity_summary(results):
    """Print theoretical endurance and decision inputs for reproducibility."""
    print(f"Nominal pack voltage: {PACK_VOLTAGE_V:.1f} V")
    print(f"Assumed flight current: {FLIGHT_CURRENT_A:.1f} A")
    for result in results:
        ideal_minutes = theoretical_flight_time_minutes(result["capacity_mah"])
        print(
            f"{result['name']}: ideal flight time {ideal_minutes:.2f} min; "
            f"capacity at jam {result['remaining_at_jam_mah']:.1f} mAh; "
            f"estimated RTH plus reserve {result['required_with_reserve_mah']:.1f} mAh; "
            f"decision: {result['decision']}"
        )


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    true_path = generate_true_path(TIME)
    rssi = generate_rssi(TIME, JAM_INDEX)

    scenarios = [
        simulate_battery_case(name, capacity, true_path, rssi)
        for name, capacity in BATTERY_CAPACITIES_MAH.items()
    ]

    print_capacity_summary(scenarios)
    plot_battery_comparison(scenarios)
    plot_trajectory_comparison(scenarios)
    plt.show()
