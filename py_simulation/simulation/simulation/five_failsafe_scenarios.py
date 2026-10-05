"""Five illustrative smart-failsafe simulation scenarios.

All trajectories and timings in this file are synthetic simulations, not physical
flight-test data. Label them as simulation results in any thesis or report.

Requirements: Python 3.9+, NumPy, Matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt


# -----------------------------------------------------------------------------
# Shared simulation configuration
# -----------------------------------------------------------------------------
JAM_TIME_S = 10.0
JAM_VALIDATION_S = 0.200
PLOT_START_S = 9.5
PLOT_END_S = 11.0
RESPONSE_SAMPLE_S = 0.001

FLIGHT_DURATION_S = 30.0
FLIGHT_SAMPLE_COUNT = 301
JAM_PATH_FRACTION = 0.78
RTH_SPEED_MPS = 3.0
DRIFT_ACCEL_BIAS_MPS2 = 0.060
DRIFT_LIMIT_M = 10.0
MANUAL_WAYPOINT_XY_M = np.array([35.0, -15.0])

plt.rcParams.update({
    "font.size": 9,
    "axes.titlesize": 11,
    "axes.labelsize": 9,
    "legend.fontsize": 8,
    "figure.titlesize": 14,
})


# -----------------------------------------------------------------------------
# Scenario simulation model
# -----------------------------------------------------------------------------
class FailsafeScenario:
    """Generate a synthetic flight route and simulate a configured jam response."""

    def __init__(self, name, preferred_action, gps_available, battery_percent,
                 scenario_kind, seed=1):
        self.name = name
        self.preferred_action = preferred_action
        self.gps_available = gps_available
        self.battery_percent = battery_percent
        self.scenario_kind = scenario_kind
        self.rng = np.random.default_rng(seed)

        self.flight_time_s = np.linspace(0.0, FLIGHT_DURATION_S, FLIGHT_SAMPLE_COUNT)
        self.route_xy = self._make_outbound_path(self.flight_time_s)
        self.jam_index = int(JAM_PATH_FRACTION * (FLIGHT_SAMPLE_COUNT - 1))
        self.jam_position = self.route_xy[self.jam_index].copy()

    @staticmethod
    def _make_outbound_path(time_s):
        """Smooth synthetic outbound route; origin is the home location."""
        x_m = 4.0 * time_s
        y_m = 22.0 * np.sin(0.055 * time_s)
        return np.column_stack((x_m, y_m))

    @staticmethod
    def _move_toward(start_xy, target_xy, speed_mps, dt_s=0.5):
        """Return sampled positions along a straight path, excluding the start."""
        start_xy = np.asarray(start_xy, dtype=float)
        target_xy = np.asarray(target_xy, dtype=float)
        distance = np.linalg.norm(target_xy - start_xy)
        if distance <= 1e-9:
            return np.empty((0, 2), dtype=float)
        step_count = max(1, int(np.ceil(distance / (speed_mps * dt_s))))
        fractions = np.linspace(1.0 / step_count, 1.0, step_count)
        return start_xy[None, :] + fractions[:, None] * (target_xy - start_xy)[None, :]

    def simulate(self):
        """Return the executed XY path and metadata for this scenario."""
        pre_jam_path = self.route_xy[: self.jam_index + 1].copy()
        post_jam_path = np.empty((0, 2), dtype=float)
        final_decision = ""
        event_position = None
        event_label = None
        manual_waypoint = None

        if self.scenario_kind == "gps_rth":
            post_jam_path = self._move_toward(self.jam_position, np.zeros(2), RTH_SPEED_MPS)
            final_decision = "GPS RTH to home"

        elif self.scenario_kind == "dr_retrace":
            # Stored route waypoints are followed in exact reverse order.
            post_jam_path = pre_jam_path[-2::-1].copy()
            final_decision = "Dead-reckoning path retrace"

        elif self.scenario_kind == "low_battery_land":
            # Immediate land is represented in the XY map by a stationary endpoint.
            post_jam_path = self.jam_position[None, :].copy()
            final_decision = "Battery override: immediate LAND"

        elif self.scenario_kind == "manual_recovery":
            halfway_home = 0.52 * self.jam_position
            dead_reckoning_leg = self._move_toward(
                self.jam_position, halfway_home, RTH_SPEED_MPS
            )
            manual_start = dead_reckoning_leg[-1] if len(dead_reckoning_leg) else self.jam_position
            manual_waypoint = MANUAL_WAYPOINT_XY_M.copy()
            manual_leg = self._move_toward(manual_start, manual_waypoint, RTH_SPEED_MPS)
            post_jam_path = np.vstack((dead_reckoning_leg, manual_leg))
            event_position = manual_start
            event_label = "RSSI recovered; manual override"
            final_decision = "Manual control to new waypoint"

        elif self.scenario_kind == "drift_abort":
            actual_position = self.jam_position.copy()
            estimate_drift = np.zeros(2)
            points = []
            elapsed_s = 0.0
            threshold_crossed = False

            # Accelerometer bias creates approximately quadratic position error.
            # The estimated position influences the commanded return direction.
            while elapsed_s < FLIGHT_DURATION_S:
                elapsed_s += 0.5
                estimate_drift[1] = 0.5 * DRIFT_ACCEL_BIAS_MPS2 * elapsed_s**2
                estimated_xy = actual_position + estimate_drift
                error_m = np.linalg.norm(estimate_drift)
                if error_m >= DRIFT_LIMIT_M:
                    threshold_crossed = True
                    event_position = actual_position.copy()
                    event_label = f"DR error reached {DRIFT_LIMIT_M:.0f} m"
                    points.append(actual_position.copy())
                    break

                vector_to_home = -estimated_xy
                distance = np.linalg.norm(vector_to_home)
                if distance <= RTH_SPEED_MPS * 0.5:
                    actual_position[:] = 0.0
                    points.append(actual_position.copy())
                    break

                actual_position += vector_to_home / distance * RTH_SPEED_MPS * 0.5
                # Small extra inertial random walk makes the displayed estimate less ideal.
                estimate_drift += self.rng.normal(0.0, 0.025, size=2)
                points.append(actual_position.copy())

            post_jam_path = np.asarray(points, dtype=float)
            final_decision = (
                "Drift threshold exceeded: emergency LAND"
                if threshold_crossed
                else "Dead-reckoning RTH"
            )

        else:
            raise ValueError(f"Unknown scenario kind: {self.scenario_kind}")

        if len(post_jam_path):
            executed_path = np.vstack((pre_jam_path, post_jam_path))
        else:
            executed_path = pre_jam_path.copy()

        return {
            "name": self.name,
            "preferred_action": self.preferred_action,
            "gps_available": self.gps_available,
            "battery_percent": self.battery_percent,
            "pre_jam_path": pre_jam_path,
            "executed_path": executed_path,
            "jam_position": self.jam_position.copy(),
            "final_decision": final_decision,
            "event_position": event_position,
            "event_label": event_label,
            "manual_waypoint": manual_waypoint,
        }


# -----------------------------------------------------------------------------
# Response-time figure
# -----------------------------------------------------------------------------
def plot_response_time():
    """Plot RSSI and packet-rate firmware states with a 200 ms validation delay."""
    packet_rate_hz = 260.0
    packet_period_s = 1.0 / packet_rate_hz
    sample_count = int(round((PLOT_END_S - PLOT_START_S) / RESPONSE_SAMPLE_S)) + 1
    time_s = np.linspace(PLOT_START_S, PLOT_END_S, sample_count)
    rssi_dbm = np.where(time_s < JAM_TIME_S, -40.0, -110.0)

    # Reflect evaluation on received packet boundaries. At 260 Hz, 200 ms is 52 packets.
    packet_time_s = np.arange(
        np.ceil(PLOT_START_S * packet_rate_hz),
        np.floor(PLOT_END_S * packet_rate_hz) + 1,
    ) / packet_rate_hz
    packet_state = np.zeros(packet_time_s.size, dtype=int)
    jam_seen = packet_time_s >= JAM_TIME_S
    validating = jam_seen & (packet_time_s < JAM_TIME_S + JAM_VALIDATION_S)
    failsafe = packet_time_s >= JAM_TIME_S + JAM_VALIDATION_S
    packet_state[validating] = 1
    packet_state[failsafe] = 2

    # Extend packet-sampled states to the high-resolution display timeline.
    sample_state = np.zeros(time_s.size, dtype=int)
    packet_indices = np.searchsorted(time_s, packet_time_s)
    packet_indices = np.clip(packet_indices, 0, len(time_s) - 1)
    state_change_indices = packet_indices[packet_state > 0]
    state_change_values = packet_state[packet_state > 0]
    for index, value in zip(state_change_indices, state_change_values):
        sample_state[index:] = value

    fig, ax_rssi = plt.subplots(figsize=(12, 6))
    ax_state = ax_rssi.twinx()

    ax_rssi.plot(time_s, rssi_dbm, color="tab:blue", linewidth=2.0, label="RSSI")
    ax_rssi.set_ylabel("RSSI (dBm)", color="tab:blue")
    ax_rssi.tick_params(axis="y", labelcolor="tab:blue")
    ax_rssi.set_ylim(-120.0, -30.0)
    ax_rssi.grid(True, alpha=0.3)

    ax_state.step(
        time_s, sample_state, where="post", color="tab:purple", linewidth=2.2,
        label="Firmware state",
    )
    ax_state.set_yticks([0, 1, 2])
    ax_state.set_yticklabels([
        "NORMAL_FLIGHT",
        "VALIDATING_JAM",
        "DEAD_RECKONING_FAILSAFE",
    ])
    ax_state.set_ylim(-0.35, 2.35)
    ax_state.set_ylabel("Microcontroller state")

    ax_rssi.axvline(
        JAM_TIME_S, color="black", linestyle="--", linewidth=1.6,
        label="Jammer activated (10.000 s)",
    )
    ax_rssi.axvline(
        JAM_TIME_S + JAM_VALIDATION_S, color="green", linestyle="--", linewidth=1.6,
        label="Failsafe triggered (10.200 s)",
    )
    ax_rssi.axvspan(JAM_TIME_S, JAM_TIME_S + JAM_VALIDATION_S, color="gold", alpha=0.16)
    ax_rssi.annotate(
        "200 ms validation",
        xy=(JAM_TIME_S + JAM_VALIDATION_S / 2.0, -77.0),
        ha="center",
        va="center",
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
    )

    ax_rssi.set_xlim(PLOT_START_S, PLOT_END_S)
    ax_rssi.set_xlabel("Time (s)")
    ax_rssi.set_title("Simulated AFHDS 2A Failsafe Response Time (260 Hz packet rate)")
    lines_a, labels_a = ax_rssi.get_legend_handles_labels()
    lines_b, labels_b = ax_state.get_legend_handles_labels()
    ax_rssi.legend(lines_a + lines_b, labels_a + labels_b, loc="lower left", ncols=2)
    fig.tight_layout()
    return fig


# -----------------------------------------------------------------------------
# Five simulated flight scenarios and trajectory figure
# -----------------------------------------------------------------------------
def build_scenarios():
    """Configure the five synthetic cases; all outputs are simulation results."""
    definitions = [
        ("Test 1: Standard GPS RTH", "RTH", True, 85.0, "gps_rth"),
        ("Test 2: Dead Reckoning Retrace", "RETRACE", False, 82.0, "dr_retrace"),
        ("Test 3: Low Battery Forced Land", "RETRACE", False, 18.0, "low_battery_land"),
        ("Test 4: Intermittent Jam / Manual", "RTH", False, 76.0, "manual_recovery"),
        ("Test 5: Drift Threshold Emergency Land", "RTH", False, 80.0, "drift_abort"),
    ]
    return [
        FailsafeScenario(name, preference, gps, battery, kind, seed=index + 1).simulate()
        for index, (name, preference, gps, battery, kind) in enumerate(definitions)
    ]


def plot_scenario_grid(results):
    """Create a 2x3 thesis-style grid containing five scenarios and a legend panel."""
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    flat_axes = axes.ravel()
    colors = ["tab:blue", "tab:green", "tab:orange", "tab:purple", "tab:red"]

    for index, (result, ax) in enumerate(zip(results, flat_axes[:5])):
        pre_jam = result["pre_jam_path"]
        path = result["executed_path"]
        color = colors[index]

        ax.plot(
            pre_jam[:, 0], pre_jam[:, 1], color="0.35", linestyle="--",
            linewidth=1.5, label="True outbound path before jam",
        )
        ax.plot(
            path[:, 0], path[:, 1], color=color, linewidth=2.2,
            label="Simulated executed trajectory",
        )
        ax.scatter(
            0.0, 0.0, color="black", marker="*", s=120, zorder=5, label="Home",
        )
        ax.scatter(
            result["jam_position"][0], result["jam_position"][1],
            color="red", marker="x", s=80, linewidths=2.0, zorder=6,
            label="Jamming point",
        )
        ax.scatter(
            path[-1, 0], path[-1, 1], color=color, marker="o", s=45,
            zorder=5, label="Final XY position",
        )

        event_position = result["event_position"]
        if event_position is not None:
            ax.scatter(
                event_position[0], event_position[1], color="goldenrod", marker="D",
                s=55, zorder=7, label=result["event_label"],
            )
        if result["manual_waypoint"] is not None:
            ax.scatter(
                result["manual_waypoint"][0], result["manual_waypoint"][1],
                color="magenta", marker="*", s=115, zorder=7,
                label="Manual waypoint",
            )

        gps_text = "Available" if result["gps_available"] else "Lost"
        details = (
            f"Preferred: {result['preferred_action']}\n"
            f"GPS after jam: {gps_text}\n"
            f"Battery at jam: {result['battery_percent']:.0f}%\n"
            f"Decision: {result['final_decision']}"
        )
        ax.text(
            0.03, 0.97, details, transform=ax.transAxes, va="top", fontsize=8.2,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
        )
        ax.set_title(result["name"])
        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(loc="lower left", fontsize=7, framealpha=0.85)

    # Sixth panel: shared interpretation and simulation disclosure.
    legend_ax = flat_axes[5]
    legend_ax.axis("off")
    legend_ax.set_title("Scenario Notes", pad=12)
    legend_text = (
        "SIMULATION ONLY\n"
        "These five cases are synthetic model outputs, not physical flight tests.\n\n"
        "1. Narrow-band control-link loss; GPS-guided RTH.\n"
        "2. Wide-band loss; retrace saved route in reverse.\n"
        "3. Low capacity; land at the jamming point.\n"
        "4. RSSI recovers; manual override diverts to a new waypoint.\n"
        "5. Estimated inertial drift crosses 10 m; abort RTH and land.\n\n"
        "Paths are illustrative and use simplified navigation and motion models."
    )
    legend_ax.text(
        0.03, 0.96, legend_text, transform=legend_ax.transAxes,
        ha="left", va="top", fontsize=10, linespacing=1.55,
        bbox={"boxstyle": "round", "facecolor": "#f3f5f7", "edgecolor": "0.75", "alpha": 1.0},
    )

    fig.suptitle("Smart Failsafe: Five Simulated Scenario Outcomes", y=0.995)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    return fig


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    simulation_results = build_scenarios()
    plot_response_time()
    plot_scenario_grid(simulation_results)
    plt.show()
