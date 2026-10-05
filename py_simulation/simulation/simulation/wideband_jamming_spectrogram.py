"""Stylized theoretical spectrogram of wideband RF jamming.

Shows weak GPS L1 and frequency-hopping control packets before a jammer event,
then a broadband barrage spanning 1.0 to 3.0 GHz from t=5 s. A short swept
component at onset illustrates a frequency sweep followed by sustained barrage.
This is synthetic visualization data, not a measured spectrum or a jammer design.

Requirements: NumPy and Matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt


# -----------------------------------------------------------------------------
# Spectrogram axes and synthetic background
# -----------------------------------------------------------------------------
RANDOM_SEED = 24
TIME_START_S = 0.0
TIME_END_S = 10.0
TIME_SAMPLES = 1001
FREQUENCY_START_GHZ = 1.0
FREQUENCY_END_GHZ = 3.0
FREQUENCY_SAMPLES = 501

JAM_TIME_S = 5.0
SWEEP_DURATION_S = 0.7
GPS_L1_GHZ = 1.575
CONTROL_CENTER_GHZ = 2.4

rng = np.random.default_rng(RANDOM_SEED)
time_s = np.linspace(TIME_START_S, TIME_END_S, TIME_SAMPLES)
frequency_ghz = np.linspace(
    FREQUENCY_START_GHZ, FREQUENCY_END_GHZ, FREQUENCY_SAMPLES
)

# Power is displayed in synthetic relative dB units. Noise is deliberately weak.
power_db = -91.0 + rng.normal(0.0, 1.2, size=(FREQUENCY_SAMPLES, TIME_SAMPLES))


def add_narrowband_signal(center_ghz, level_db, bandwidth_ghz, active_time_mask):
    """Add a Gaussian-shaped narrowband carrier over selected time samples."""
    profile = np.exp(
        -0.5 * ((frequency_ghz - center_ghz) / bandwidth_ghz) ** 2
    )
    selected_columns = np.flatnonzero(active_time_mask)
    for column in selected_columns:
        power_db[:, column] = np.maximum(
            power_db[:, column], -91.0 + level_db * profile
        )


# GPS L1: a weak, continuous carrier visible until the jammer activates.
pre_jam = time_s < JAM_TIME_S
add_narrowband_signal(
    GPS_L1_GHZ,
    level_db=22.0,
    bandwidth_ghz=0.012,
    active_time_mask=pre_jam,
)

# Control link: short packets hop among channels in the 2.4 GHz ISM band.
packet_starts_s = np.arange(0.15, JAM_TIME_S, 0.24)
hop_frequencies_ghz = rng.uniform(2.38, 2.48, size=len(packet_starts_s))
for packet_start_s, hop_frequency_ghz in zip(packet_starts_s, hop_frequencies_ghz):
    packet_mask = (time_s >= packet_start_s) & (time_s < packet_start_s + 0.075)
    add_narrowband_signal(
        hop_frequency_ghz,
        level_db=34.0,
        bandwidth_ghz=0.009,
        active_time_mask=packet_mask,
    )

# Wideband barrage begins at 5 s and fills the full displayed spectrum. The
# rapid ramp gives a visible onset; a swept ridge overlays its first 0.7 s.
jam_mask = time_s >= JAM_TIME_S
ramp = np.clip((time_s - JAM_TIME_S) / 0.12, 0.0, 1.0)
barrage_level_db = -91.0 + 69.0 * ramp
power_db[:, jam_mask] = np.maximum(
    power_db[:, jam_mask], barrage_level_db[jam_mask][None, :]
)

sweep_mask = (time_s >= JAM_TIME_S) & (time_s <= JAM_TIME_S + SWEEP_DURATION_S)
sweep_progress = (time_s[sweep_mask] - JAM_TIME_S) / SWEEP_DURATION_S
sweep_center_ghz = (
    FREQUENCY_START_GHZ
    + sweep_progress * (FREQUENCY_END_GHZ - FREQUENCY_START_GHZ)
)
for column, center_ghz in zip(np.flatnonzero(sweep_mask), sweep_center_ghz):
    sweep_profile = 76.0 * np.exp(
        -0.5 * ((frequency_ghz - center_ghz) / 0.035) ** 2
    )
    power_db[:, column] = np.maximum(power_db[:, column], -91.0 + sweep_profile)

# -----------------------------------------------------------------------------
# Plot
# -----------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(13, 7))
mesh = ax.pcolormesh(
    time_s,
    frequency_ghz,
    power_db,
    shading="auto",
    cmap="inferno",
    vmin=-95.0,
    vmax=-20.0,
)

# Carrier guides are shown only in the pre-jam interval to emphasize that they
# are overwhelmed once the broadband jammer turns on.
ax.plot(
    [TIME_START_S, JAM_TIME_S],
    [GPS_L1_GHZ, GPS_L1_GHZ],
    color="cyan",
    linestyle="--",
    linewidth=1.6,
    label="GPS L1 carrier (1.575 GHz; pre-jam)",
)
ax.plot(
    [TIME_START_S, JAM_TIME_S],
    [CONTROL_CENTER_GHZ, CONTROL_CENTER_GHZ],
    color="lime",
    linestyle=":",
    linewidth=1.5,
    label="2.4 GHz control-hop band (pre-jam)",
)

ax.axvline(
    JAM_TIME_S,
    color="white",
    linestyle="--",
    linewidth=1.8,
    label="Wideband jammer activated",
)
ax.axvspan(JAM_TIME_S, TIME_END_S, color="red", alpha=0.06)
ax.text(
    JAM_TIME_S + 0.15,
    FREQUENCY_END_GHZ - 0.08,
    "Broadband barrage: 1.0-3.0 GHz",
    color="white",
    fontsize=10,
    ha="left",
    va="top",
    bbox={"boxstyle": "round", "facecolor": "black", "alpha": 0.55},
)
ax.text(
    JAM_TIME_S + 0.35,
    2.02,
    "Illustrative sweep at onset",
    color="white",
    fontsize=9,
    rotation=52,
    ha="center",
    va="center",
)

colorbar = fig.colorbar(mesh, ax=ax, pad=0.02)
colorbar.set_label("Relative received power (dB, synthetic)")
ax.set_title("Theoretical Wideband Jamming Across GPS L1 and 2.4 GHz Control Band")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Frequency (GHz)")
ax.set_xlim(TIME_START_S, TIME_END_S)
ax.set_ylim(FREQUENCY_START_GHZ, FREQUENCY_END_GHZ)
ax.set_xticks(np.arange(TIME_START_S, TIME_END_S + 0.1, 1.0))
ax.set_yticks(np.arange(FREQUENCY_START_GHZ, FREQUENCY_END_GHZ + 0.01, 0.25))
ax.grid(True, color="white", alpha=0.12, linewidth=0.6)
ax.legend(loc="lower left", framealpha=0.9, fontsize=9)
fig.tight_layout()

if __name__ == "__main__":
    plt.show()
