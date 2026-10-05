"""Millisecond-resolution AFHDS 2A jamming and failsafe timing visualization.

This is an illustrative firmware timing model, not a measurement of a specific
receiver or flight-controller implementation. It models 260 packet updates/s,
a sudden RF jam at 10.000 s, and a 200 ms loss-validation interval.

Dependencies: NumPy and Matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator


# -----------------------------------------------------------------------------
# Protocol and simulation parameters
# -----------------------------------------------------------------------------
PACKET_RATE_HZ = 260.0
PACKET_PERIOD_S = 1.0 / PACKET_RATE_HZ
JAM_TIME_S = 10.000
VALIDATION_WINDOW_S = 0.200
FAILSAFE_TIME_S = JAM_TIME_S + VALIDATION_WINDOW_S
WINDOW_START_S = 9.500
WINDOW_END_S = 10.500

RSSI_BEFORE_DBM = -45.0
RSSI_DURING_JAM_DBM = -110.0
RSSI_NOISE_STD_DB = 0.35

RSSI_LOSS_THRESHOLD_DBM = -105.0
CRC_BITS = 16  # AFHDS 2A packet CRC width, shown for protocol context.
RANDOM_SEED = 17


# -----------------------------------------------------------------------------
# Generate packet-rate telemetry over the focused one-second interval
# -----------------------------------------------------------------------------
def generate_timeline():
    """Return packet timestamps, sensor traces, loss metrics, and firmware states."""
    first_packet = int(np.ceil(WINDOW_START_S * PACKET_RATE_HZ))
    last_packet = int(np.floor(WINDOW_END_S * PACKET_RATE_HZ))
    packet_numbers = np.arange(first_packet, last_packet + 1)
    time_s = packet_numbers / PACKET_RATE_HZ

    rng = np.random.default_rng(RANDOM_SEED)
    jam_active = time_s >= JAM_TIME_S

    rssi_dbm = np.where(jam_active, RSSI_DURING_JAM_DBM, RSSI_BEFORE_DBM)
    rssi_dbm = rssi_dbm + rng.normal(0.0, RSSI_NOISE_STD_DB, size=time_s.size)
    # Keep the synthetic RSSI cleanly on either side of the modeled threshold.
    rssi_dbm[~jam_active] = np.maximum(rssi_dbm[~jam_active], RSSI_LOSS_THRESHOLD_DBM + 1.0)
    rssi_dbm[jam_active] = np.minimum(rssi_dbm[jam_active], RSSI_LOSS_THRESHOLD_DBM - 1.0)

    # A wide-band jam corrupts/drops every packet after activation in this model.
    packet_loss_percent = np.where(jam_active, 100.0, 0.0)
    crc_error_percent = np.where(jam_active, 100.0, 0.0)

    # State evaluation occurs on packet arrivals. 260 Hz * 200 ms = 52 packets,
    # so the validation interval and transition are exactly representable here.
    loss_detected = rssi_dbm < RSSI_LOSS_THRESHOLD_DBM
    validation_elapsed_s = time_s - JAM_TIME_S
    state = np.zeros(time_s.size, dtype=int)  # 0: NORMAL_FLIGHT
    state[loss_detected & (validation_elapsed_s < VALIDATION_WINDOW_S)] = 1
    state[validation_elapsed_s >= VALIDATION_WINDOW_S] = 2  # DEAD_RECKONING_FAILSAFE

    return time_s, rssi_dbm, packet_loss_percent, crc_error_percent, state


# -----------------------------------------------------------------------------
# Plotting
# -----------------------------------------------------------------------------
def plot_failsafe_timing():
    """Create three aligned plots focused on the jam and firmware response."""
    time_s, rssi, packet_loss, crc_errors, states = generate_timeline()
    time_ms = (time_s - JAM_TIME_S) * 1000.0
    jam_ms = 0.0
    failsafe_ms = VALIDATION_WINDOW_S * 1000.0

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(13, 9),
        sharex=True,
        gridspec_kw={"height_ratios": [1.0, 1.0, 1.2]},
    )
    ax_signal, ax_integrity, ax_state = axes

    # 1. Signal strength
    ax_signal.plot(time_ms, rssi, color="tab:blue", linewidth=1.8, label="RSSI")
    ax_signal.axhline(
        RSSI_LOSS_THRESHOLD_DBM,
        color="gray",
        linestyle=":",
        linewidth=1.2,
        label=f"Loss threshold ({RSSI_LOSS_THRESHOLD_DBM} dBm)",
    )
    ax_signal.set_ylabel("RSSI (dBm)")
    ax_signal.set_ylim(-118, -38)
    ax_signal.set_title("AFHDS 2A Jamming Response at 260 Packets/s")
    ax_signal.legend(loc="lower left", ncols=2, fontsize=9)
    ax_signal.grid(True, alpha=0.3)

    # 2. Packet and CRC integrity rates
    ax_integrity.step(
        time_ms, packet_loss, where="post", color="tab:orange", linewidth=1.8,
        label="Packet loss (%)",
    )
    ax_integrity.step(
        time_ms, crc_errors, where="post", color="tab:red", linewidth=1.5,
        linestyle="--", label=f"CRC error rate (%) - {CRC_BITS}-bit CRC",
    )
    ax_integrity.set_ylabel("Error / loss (%)")
    ax_integrity.set_ylim(-5, 108)
    ax_integrity.set_yticks([0, 25, 50, 75, 100])
    ax_integrity.legend(loc="lower right", fontsize=9)
    ax_integrity.grid(True, alpha=0.3)

    # 3. Firmware state machine. Categorical states are encoded for a readable step plot.
    ax_state.step(time_ms, states, where="post", color="tab:purple", linewidth=2.2)
    ax_state.set_yticks([0, 1, 2])
    ax_state.set_yticklabels([
        "NORMAL_FLIGHT",
        "VALIDATING_LOSS",
        "DEAD_RECKONING_FAILSAFE",
    ])
    ax_state.set_ylim(-0.35, 2.35)
    ax_state.set_ylabel("Firmware state")
    ax_state.set_xlabel("Time relative to jamming event (ms) - absolute jam time: 10.000 s")
    ax_state.grid(True, alpha=0.3)

    # Jam and validated-failsafe markers appear on every subplot for alignment.
    for ax in axes:
        ax.axvspan(jam_ms, failsafe_ms, color="gold", alpha=0.12, zorder=0)
        ax.axvline(jam_ms, color="black", linestyle="--", linewidth=1.5)
        ax.axvline(failsafe_ms, color="green", linestyle="--", linewidth=1.5)
        ax.set_xlim((WINDOW_START_S - JAM_TIME_S) * 1000.0,
                    (WINDOW_END_S - JAM_TIME_S) * 1000.0)
        ax.xaxis.set_major_locator(MultipleLocator(100))
        ax.xaxis.set_minor_locator(MultipleLocator(25))
        ax.grid(which="minor", axis="x", alpha=0.12)

    ax_signal.text(
        0.01, 0.95, "Jammer activated: 10.000 s", transform=ax_signal.transAxes,
        va="top", ha="left", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )
    ax_state.text(
        0.99, 0.95, "Failsafe triggered: 10.200 s (200 ms validation)",
        transform=ax_state.transAxes, va="top", ha="right", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )

    fig.tight_layout()
    return fig


if __name__ == "__main__":
    plot_failsafe_timing()
    plt.show()
