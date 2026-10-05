"""Simulate 30 seconds of GPS-denied inertial dead reckoning during RTH.

The ground-truth aircraft follows a smooth return-to-home path. The estimated
position is obtained by double-integrating accelerometer measurements containing
constant bias and Gaussian noise. This simplified model illustrates drift; it is
not a validated flight-control or MPU6500 performance model.

Requirements: Python 3.9+, NumPy, Matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt


# -----------------------------------------------------------------------------
# Simulation configuration
# -----------------------------------------------------------------------------
DURATION_S = 30.0
SAMPLE_RATE_HZ = 100.0
DT = 1.0 / SAMPLE_RATE_HZ
TIME_S = np.arange(0.0, DURATION_S + DT, DT)
RANDOM_SEED = 6500

# Initial position is the last GPS fix at the instant GPS is lost. The drone
# returns toward home (0, 0) during the simulated 30-second failsafe maneuver.
START_XY_M = np.array([120.0, 35.0])
HOME_XY_M = np.array([0.0, 0.0])

# Illustrative MPU6500-like errors in the horizontal navigation frame.
# Bias is constant; white noise is sampled independently on each axis.
ACCEL_BIAS_MPS2 = np.array([0.018, -0.012])
ACCEL_NOISE_STD_MPS2 = 0.045


# -----------------------------------------------------------------------------
# Ground truth, synthetic IMU, and dead reckoning
# -----------------------------------------------------------------------------
def generate_ground_truth(time_s):
    """Generate a smooth 2D RTH path with zero initial and final velocity."""
    progress = np.clip(time_s / DURATION_S, 0.0, 1.0)
    smooth_progress = 3.0 * progress**2 - 2.0 * progress**3
    return START_XY_M[None, :] + smooth_progress[:, None] * (HOME_XY_M - START_XY_M)[None, :]


def calculate_acceleration(position_xy, dt_s):
    """Estimate horizontal acceleration from the ground-truth position sequence."""
    velocity_xy = np.gradient(position_xy, dt_s, axis=0, edge_order=2)
    acceleration_xy = np.gradient(velocity_xy, dt_s, axis=0, edge_order=2)
    return acceleration_xy


def integrate_noisy_imu(true_acceleration_xy, dt_s, rng):
    """Double-integrate accelerometer samples from the last known GPS position."""
    noise = rng.normal(
        loc=0.0,
        scale=ACCEL_NOISE_STD_MPS2,
        size=true_acceleration_xy.shape,
    )
    measured_acceleration_xy = (
        true_acceleration_xy + ACCEL_BIAS_MPS2[None, :] + noise
    )

    estimated_position_xy = np.empty_like(true_acceleration_xy)
    estimated_velocity_xy = np.zeros_like(true_acceleration_xy)
    estimated_position_xy[0] = START_XY_M

    # The initial position and velocity are assumed known at GPS-loss time.
    for index in range(1, len(true_acceleration_xy)):
        estimated_velocity_xy[index] = (
            estimated_velocity_xy[index - 1]
            + measured_acceleration_xy[index - 1] * dt_s
        )
        estimated_position_xy[index] = (
            estimated_position_xy[index - 1]
            + estimated_velocity_xy[index] * dt_s
        )

    return estimated_position_xy


def theoretical_bias_error(time_s):
    """Position error magnitude caused by constant accel bias alone (0.5*a*t^2)."""
    bias_magnitude = np.linalg.norm(ACCEL_BIAS_MPS2)
    return 0.5 * bias_magnitude * time_s**2


# -----------------------------------------------------------------------------
# Visualization
# -----------------------------------------------------------------------------
def plot_dead_reckoning_results():
    rng = np.random.default_rng(RANDOM_SEED)
    ground_truth_xy = generate_ground_truth(TIME_S)
    true_acceleration_xy = calculate_acceleration(ground_truth_xy, DT)
    estimated_xy = integrate_noisy_imu(true_acceleration_xy, DT, rng)

    position_error_m = np.linalg.norm(estimated_xy - ground_truth_xy, axis=1)
    bias_reference_error_m = theoretical_bias_error(TIME_S)

    fig, (ax_path, ax_error) = plt.subplots(
        1,
        2,
        figsize=(14, 6),
        gridspec_kw={"width_ratios": [1.2, 1.0]},
    )

    # Left: XY ground truth versus the IMU-only estimate.
    ax_path.plot(
        ground_truth_xy[:, 0],
        ground_truth_xy[:, 1],
        color="black",
        linewidth=2.4,
        label="Ground truth",
    )
    ax_path.plot(
        estimated_xy[:, 0],
        estimated_xy[:, 1],
        color="tab:blue",
        linewidth=1.8,
        label="Dead-reckoning estimate",
    )
    ax_path.scatter(
        START_XY_M[0], START_XY_M[1],
        color="tab:green", marker="o", s=75, zorder=4, label="GPS lost / initial fix",
    )
    ax_path.scatter(
        HOME_XY_M[0], HOME_XY_M[1],
        color="tab:red", marker="*", s=150, zorder=4, label="Home",
    )

    # Mark the estimated position at the end of the 30-second outage.
    ax_path.annotate(
        f"30 s estimate\nerror = {position_error_m[-1]:.1f} m",
        xy=estimated_xy[-1],
        xytext=(12, 12),
        textcoords="offset points",
        fontsize=9,
        arrowprops={"arrowstyle": "->", "color": "tab:blue"},
    )
    ax_path.set_title("GPS-Denied RTH Trajectory")
    ax_path.set_xlabel("East position (m)")
    ax_path.set_ylabel("North position (m)")
    ax_path.axis("equal")
    ax_path.grid(True, alpha=0.35)
    ax_path.legend(loc="best", fontsize=9)

    # Right: actual position-error magnitude and the constant-bias reference.
    ax_error.plot(
        TIME_S,
        position_error_m,
        color="tab:blue",
        linewidth=2.0,
        label="Dead-reckoning position error",
    )
    ax_error.plot(
        TIME_S,
        bias_reference_error_m,
        color="tab:orange",
        linestyle="--",
        linewidth=2.0,
        label="Constant-bias reference: 0.5 |b| t^2",
    )
    ax_error.set_title("Position Error During 30 s GPS Outage")
    ax_error.set_xlabel("Time since GPS loss (s)")
    ax_error.set_ylabel("Position error magnitude (m)")
    ax_error.set_xlim(0.0, DURATION_S)
    ax_error.set_ylim(bottom=0.0)
    ax_error.grid(True, alpha=0.35)
    ax_error.legend(loc="upper left", fontsize=9)

    # GPS is unavailable from the initial sample; mark outage start on the timeline.
    ax_error.axvline(
        0.0, color="tab:red", linestyle=":", linewidth=1.8, label="GPS lost"
    )
    ax_error.annotate(
        "GPS lost",
        xy=(0.0, position_error_m[0]),
        xytext=(12, 18),
        textcoords="offset points",
        color="tab:red",
        fontsize=9,
        arrowprops={"arrowstyle": "->", "color": "tab:red"},
    )
    ax_path.annotate(
        "GPS lost: estimate initialized at last fix",
        xy=START_XY_M,
        xytext=(0.04, 0.96),
        textcoords="axes fraction",
        ha="left",
        va="top",
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
    )

    fig.suptitle(
        "MPU6500-Style Inertial Drift During Dead-Reckoning Return to Home",
        fontsize=14,
    )
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    plot_dead_reckoning_results()
    plt.show()
