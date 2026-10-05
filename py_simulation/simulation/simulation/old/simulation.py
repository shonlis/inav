"""
UAV Failsafe Simulation
- Generates synthetic sensor streams (RSSI, GPS availability, IMU, battery)
- Implements a frame-by-frame state machine (Normal -> JamDetected -> DeadReck -> Failsafe -> Landed)
- Dead-reckoning integrates noisy IMU (accel + yaw gyro)
- Failsafe modes: 'land', 'rth' (return-to-home), 'retrace' (reverse stored waypoints)
- Produces 3 subplots: Signal & Battery, State transitions, 2D Trajectory (true vs DR vs Failsafe)
"""

import numpy as np
import matplotlib.pyplot as plt

# ----------------------
# Configurable parameters
# ----------------------
SIM_SECONDS = 100.0
HZ = 10.0
DT = 1.0 / HZ
N = int(SIM_SECONDS * HZ)
SEED = 42

RSSI_MEAN_BEFORE = -40.0  # dBm
RSSI_NOISE_STD = 1.0
RSSI_SENSITIVITY = -105.0  # dBm threshold
JAM_TIME = 40.0  # seconds when jamming starts

BATTERY_START = 100.0  # %
BATTERY_END = 20.0  # % at end of sim

# IMU noise
ACCEL_NOISE_STD = 0.12  # m/s^2
GYRO_Z_NOISE_STD = 0.01  # rad/s

# Trajectory params (true path)
CRUISE_SPEED = 4.0  # m/s
CURVE_AMPLITUDE = 8.0
CURVE_FREQ = 0.06

# Failsafe params
FAIL_DELAY = 3.0  # seconds in Dead Reckoning before starting Failsafe
LAND_DESCEND_RATE = 0.6  # m/s
FAILSAFE_MODE = 'rth'  # 'land' | 'rth' | 'retrace'

# Navigation tuning
FS_CRUISE_SPEED = 3.0  # m/s when moving in failsafe modes
WAYPOINT_SAVE_INTERVAL = 1.0  # seconds between saved waypoints

np.random.seed(SEED)


# ----------------------
# Synthetic data functions
# ----------------------
def generate_timebase(N, dt):
    return np.arange(N) * dt


def generate_true_trajectory(t):
    # Simple 2D path: x increases nearly linearly, y is a gentle sinusoidal curve
    x = CRUISE_SPEED * t
    y = CURVE_AMPLITUDE * np.sin(CURVE_FREQ * t)
    z = np.full_like(x, 10.0)  # constant altitude 10 m
    pos = np.vstack((x, y, z)).T  # shape (N,3)

    # velocities from derivatives
    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    vz = np.gradient(z, t)
    vel = np.vstack((vx, vy, vz)).T

    # accelerations from derivatives
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    az = np.gradient(vz, t)
    acc = np.vstack((ax, ay, az)).T

    # yaw (heading) from velocity vector (ignore vertical)
    yaw = np.arctan2(vy, vx)
    yaw_rate = np.gradient(yaw, t)

    return pos, vel, acc, yaw, yaw_rate


def generate_rssi_and_gps(t, jam_time):
    rssi = RSSI_MEAN_BEFORE + np.random.randn(len(t)) * RSSI_NOISE_STD
    gps_ok = np.ones(len(t), dtype=bool)
    # At jam_time, RSSI drops below sensitivity and GPS lost
    jam_index = int(jam_time / DT)
    if jam_index < len(t):
        rssi[jam_index:] = RSSI_SENSITIVITY - 5.0 + np.random.randn(len(t) - jam_index) * 1.0
        gps_ok[jam_index:] = False
    return rssi, gps_ok, jam_index


def generate_imu_measurements(true_acc, true_yaw_rate):
    # Body-frame acceleration approximation:
    # Assuming small pitch/roll so accelerometers measure horizontal acceleration in world frame
    # Add noise to accelerometer and gyro_z
    accel_meas = true_acc + np.random.randn(*true_acc.shape) * ACCEL_NOISE_STD
    gyro_z_meas = true_yaw_rate + np.random.randn(len(true_yaw_rate)) * GYRO_Z_NOISE_STD
    return accel_meas, gyro_z_meas


def generate_battery_level(t):
    # Linearly decay battery from start to end
    return np.linspace(BATTERY_START, BATTERY_END, len(t))


# ----------------------
# State machine
# ----------------------
class UAVStateMachine:
    """
    States:
      0 - Normal Flight (uses GPS)
      1 - Jam Detected (detection event)
      2 - Dead Reckoning (integrating IMU)
      3 - Failsafe Execution (land, rth, retrace)
      4 - Landed
    """

    def __init__(self, dt, failsafe_mode='land'):
        self.dt = dt
        self.state = 0
        self.failsafe_mode = failsafe_mode
        # Estimates
        self.pos_est = np.zeros(3)  # x,y,z
        self.vel_est = np.zeros(3)
        self.yaw_est = 0.0
        # store time of jam detection
        self.jam_detected_time = None
        # saved waypoints while GPS is available
        self.waypoints = []
        self.last_waypoint_saved_t = -1e9

        # For plotting/debug
        self.state_history = []
        self.est_history = []
        self.fail_exec_flags = np.zeros(N, dtype=bool)

        # internal flags for failsafe progress
        self.target_idx_retrace = None
        self.rth_active_goal = np.array([0.0, 0.0, 0.0])  # home

    def step(self, k, t_k, gps_ok, gps_pos, accel_meas, gyro_z_meas, battery_level):
        """
        Process one time-step with sensor inputs.
        Returns estimated position after processing the frame.
        """
        # Save state history
        self.state_history.append(self.state)

        # If Normal Flight, trust GPS for estimate
        if self.state == 0:
            # Use GPS perfect pose
            self.pos_est = gps_pos.copy()
            # estimate velocity from differences could be done but use gps -> velocity from motion
            # reset yaw to true yaw from velocity if available
            # Save waypoint occasionally
            if t_k - self.last_waypoint_saved_t >= WAYPOINT_SAVE_INTERVAL:
                self.waypoints.append(self.pos_est.copy())
                self.last_waypoint_saved_t = t_k

        # Jam Detection transition
        if self.state == 0 and (not gps_ok):
            # RSSI also should be low, caller ensures gps_ok implies jam condition
            self.state = 1
            self.jam_detected_time = t_k

        # Jam Detected: move to Dead Reckoning immediately
        if self.state == 1:
            # Initialize DR state from last GPS known good estimate
            self.vel_est = np.zeros(3)  # we could set from gps diff but simpler
            self.yaw_est = np.arctan2(0.0, 1.0)  # keep heading if unknown; will integrate gyro
            self.state = 2

        # Dead Reckoning: integrate IMU
        if self.state == 2:
            # Integrate yaw from gyro z
            self.yaw_est += gyro_z_meas * self.dt
            # Rotate measured accel (approx world frame accel because we consider yaw only)
            # Construct rotation matrix for yaw
            cy = np.cos(self.yaw_est)
            sy = np.sin(self.yaw_est)
            R = np.array([[cy, -sy, 0],
                          [sy, cy, 0],
                          [0, 0, 1]])
            # If accel_meas is already in world frame approximation, use directly.
            # To be slightly realistic, treat accel_meas as body-frame and rotate to world:
            accel_world = R @ accel_meas
            # Integrate velocity and position
            self.vel_est += accel_world * self.dt
            self.pos_est += self.vel_est * self.dt

            # After FAIL_DELAY seconds since jam detection, move to failsafe execution
            if (self.jam_detected_time is not None) and (t_k - self.jam_detected_time >= FAIL_DELAY):
                self.state = 3
                # Prepare failsafe specifics
                if self.failsafe_mode == 'retrace':
                    # reverse waypoints for retracing (use copy)
                    self.retrace_waypoints = list(reversed(self.waypoints))
                    self.target_idx_retrace = 0
                elif self.failsafe_mode == 'rth':
                    self.rth_active_goal = np.array([0.0, 0.0, 0.0])  # home pos (ground level)
                # keep executing in this frame as part of failsafe

        # Failsafe execution
        if self.state == 3:
            self.fail_exec_flags[k] = True  # mark for plotting
            if self.failsafe_mode == 'land':
                # Hold XY, descend
                self.vel_est[0] = 0.0
                self.vel_est[1] = 0.0
                self.vel_est[2] = -LAND_DESCEND_RATE
                self.pos_est += self.vel_est * self.dt
                # If below ground, clamp and land
                if self.pos_est[2] <= 0.0:
                    self.pos_est[2] = 0.0
                    self.vel_est[:] = 0.0
                    self.state = 4

            elif self.failsafe_mode == 'rth':
                # Move towards home horizontally at FS_CRUISE_SPEED and then land when close
                vec_to_home = self.rth_active_goal - self.pos_est
                dist = np.linalg.norm(vec_to_home[:2])
                if dist > 0.5:
                    direction = vec_to_home[:2] / dist
                    vx = direction[0] * FS_CRUISE_SPEED
                    vy = direction[1] * FS_CRUISE_SPEED
                    # maintain current altitude until near home, then descend
                    self.vel_est[0] = vx
                    self.vel_est[1] = vy
                    self.vel_est[2] = 0.0
                    self.pos_est += self.vel_est * self.dt
                else:
                    # start landing
                    self.vel_est[0:2] = 0.0
                    self.vel_est[2] = -LAND_DESCEND_RATE
                    self.pos_est += self.vel_est * self.dt
                    if self.pos_est[2] <= 0.0:
                        self.pos_est[2] = 0.0
                        self.vel_est[:] = 0.0
                        self.state = 4

            elif self.failsafe_mode == 'retrace':
                # Follow reversed waypoints from stored list using estimated position
                if self.target_idx_retrace is None or self.target_idx_retrace >= len(self.retrace_waypoints):
                    # no waypoints or completed -> land at current spot
                    self.vel_est[0:2] = 0.0
                    self.vel_est[2] = -LAND_DESCEND_RATE
                    self.pos_est += self.vel_est * self.dt
                    if self.pos_est[2] <= 0.0:
                        self.pos_est[2] = 0.0
                        self.vel_est[:] = 0.0
                        self.state = 4
                else:
                    wp = self.retrace_waypoints[self.target_idx_retrace]
                    vec = wp - self.pos_est
                    dist = np.linalg.norm(vec[:2])
                    if dist < 0.8:
                        # reached this waypoint, go to next
                        self.target_idx_retrace += 1
                    else:
                        dir2 = vec[:2] / (dist + 1e-9)
                        self.vel_est[0] = dir2[0] * FS_CRUISE_SPEED
                        self.vel_est[1] = dir2[1] * FS_CRUISE_SPEED
                        self.vel_est[2] = 0.0
                        self.pos_est += self.vel_est * self.dt

            else:
                # Unknown failsafe -> land as safe default
                self.vel_est[0:2] = 0.0
                self.vel_est[2] = -LAND_DESCEND_RATE
                self.pos_est += self.vel_est * self.dt
                if self.pos_est[2] <= 0.0:
                    self.pos_est[2] = 0.0
                    self.vel_est[:] = 0.0
                    self.state = 4

        # If Landed, hold position
        if self.state == 4:
            self.vel_est[:] = 0.0
            self.pos_est[2] = 0.0

        # Save estimate history
        self.est_history.append(self.pos_est.copy())

        return self.pos_est.copy()


# ----------------------
# Main simulation driver
# ----------------------
def run_simulation(failsafe_mode=FAILSAFE_MODE):
    t = generate_timebase(N, DT)
    true_pos, true_vel, true_acc, true_yaw, true_yaw_rate = generate_true_trajectory(t)
    rssi, gps_ok, jam_index = generate_rssi_and_gps(t, JAM_TIME)
    accel_meas, gyro_z_meas = generate_imu_measurements(true_acc, true_yaw_rate)
    battery = generate_battery_level(t)

    uav = UAVStateMachine(DT, failsafe_mode=failsafe_mode)
    est_positions = np.zeros_like(true_pos)
    states = np.zeros(N, dtype=int)

    for k in range(N):
        gps_available = gps_ok[k]
        gps_pos = true_pos[k] if gps_available else None
        # For convenience, if gps not available, pass last known gps pos or current est
        if gps_pos is None:
            gps_pos = uav.pos_est.copy()

        est = uav.step(k, t[k], gps_available, gps_pos,
                       accel_meas[k], gyro_z_meas[k],
                       battery[k])
        est_positions[k] = est
        states[k] = uav.state

    return {
        't': t,
        'true_pos': true_pos,
        'est_pos': np.array(uav.est_history),
        'rssi': rssi,
        'battery': battery,
        'states': np.array(uav.state_history),
        'jam_index': jam_index,
        'fail_exec_flags': uav.fail_exec_flags,
        'waypoints': np.array(uav.waypoints)
    }


# ----------------------
# Plotting utilities
# ----------------------
def plot_results(results, failsafe_mode):
    t = results['t']
    true_pos = results['true_pos']
    est_pos = results['est_pos']
    rssi = results['rssi']
    battery = results['battery']
    states = results['states']
    jam_index = results['jam_index']
    fail_exec_flags = results['fail_exec_flags']

    fig, axes = plt.subplots(3, 1, figsize=(11, 14))
    ax1, ax2, ax3 = axes

    # Graph 1: RSSI and Battery, dual axis
    ax1.plot(t, rssi, color='C0', label='RSSI (dBm)')
    ax1.axvline(t[jam_index], color='red', linestyle='--', label='Jamming Start')
    ax1.set_ylabel('RSSI (dBm)', color='C0')
    ax1.tick_params(axis='y', labelcolor='C0')
    ax1.grid(True)

    ax1b = ax1.twinx()
    ax1b.plot(t, battery, color='C1', label='Battery (%)')
    ax1b.set_ylabel('Battery (%)', color='C1')
    ax1b.tick_params(axis='y', labelcolor='C1')

    ax1.set_title('Signal (RSSI) and Battery')
    ax1.legend(loc='upper left')
    ax1b.legend(loc='upper right')

    # Graph 2: State transitions (step)
    # Map numeric states to names for y-ticks
    state_names = {0: 'Normal', 1: 'JamDetected', 2: 'DeadReck', 3: 'Failsafe', 4: 'Landed'}
    ax2.step(t, states, where='post')
    ax2.set_yticks(list(state_names.keys()))
    ax2.set_yticklabels([state_names[k] for k in state_names.keys()])
    ax2.set_ylim(-0.5, 4.5)
    ax2.set_title('State Machine Transitions')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True)
    ax2.axvline(t[jam_index], color='red', linestyle='--')

    # Graph 3: Trajectory - 2D XY map
    ax3.plot(true_pos[:, 0], true_pos[:, 1], '-k', linewidth=1.2, label='True Path')
    ax3.plot(est_pos[:, 0], est_pos[:, 1], '--', color='orange', linewidth=1.1, label='Dead Reckoning Estimate')
    # Mark start, jam point, and home
    ax3.scatter([true_pos[0, 0]], [true_pos[0, 1]], marker='o', color='green', label='Home / Start')
    ax3.scatter([true_pos[jam_index, 0]], [true_pos[jam_index, 1]], marker='x', color='red', label='Jam Location')

    # Indicate failsafe path segments (where fail_exec_flags True)
    if np.any(fail_exec_flags):
        idxs = np.where(fail_exec_flags)[0]
        # plot segment of estimated path during failsafe
        ax3.plot(est_pos[idxs, 0], est_pos[idxs, 1], color='C3', linewidth=2.0, label=f'Failsafe ({failsafe_mode}) Path')

    ax3.set_title('Trajectory (XY)')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.axis('equal')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    plt.show()


# ----------------------
# Run
# ----------------------
if __name__ == '__main__':
    # Swap the mode here before running: 'land' | 'rth' | 'retrace'
    for mode in ['land', 'rth', 'retrace']:
        print(f"Running simulation with FAILSAFE_MODE = '{mode}'")
        results = run_simulation(failsafe_mode=mode)
        plot_results(results, mode)
