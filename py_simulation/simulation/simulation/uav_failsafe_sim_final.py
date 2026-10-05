"""
UAV Smart Failsafe Simulation (4 Scenarios)
- Object-oriented, modular simulation using numpy + matplotlib.
- Simulates RSSI, GPS availability, battery, power-saving mode, IMU-dead-reckoning drift.
- Implements decision logic to choose between user-preferred failsafe and feasible fallback:
    LAND, RTH (GPS or DR), RETRACE (GPS or DR), Manual override on signal recovery.
- Produces:
    * Figure 1: Telemetry & Power-Saving Boolean for Scenario 4 (Recovery).
    * Figure 2: 2x2 grid of separated 2D Trajectories (one subplot per scenario).
Requirements: Python 3.7+, numpy, matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------------
SIM_SECONDS = 150.0
HZ = 10.0
DT = 1.0 / HZ
N = int(SIM_SECONDS * HZ)
TIME = np.arange(N) * DT

SEED = 1
np.random.seed(SEED)

# Physical / path params
CRUISE_SPEED = 4.0           # m/s along X
CURVE_AMP = 6.0              # lateral amplitude
CURVE_FREQ = 0.05            # lateral frequency
START_ALT = 15.0             # meters

# RSSI/jamming
RSSI_STRONG = -40.0
RSSI_NOISE_STD = 0.8
RSSI_SENSITIVITY = -105.0
JAM_TIME = 40.0
JAM_IDX = int(JAM_TIME / DT)

# For scenario 4 recovery: RSSI returns at this time
RECOVERY_TIME = 80.0
RECOVERY_IDX = int(RECOVERY_TIME / DT)

# Battery model (percent per second)
BATTERY_DRAIN_FLY = 0.18     # %/s while cruising
BATTERY_DRAIN_FS = 0.12      # %/s while performing failsafe movement
BATTERY_DRAIN_DESCEND = 0.06 # %/s while descending
BATTERY_SAFETY_MARGIN = 3.0  # percent reserve required

# Failsafe motion parameters
FS_SPEED = 3.0               # m/s horizontal during failsafe
DESCEND_RATE = 0.6           # m/s descent rate for landing

# Waypoint saving
WAYPOINT_INTERVAL_S = 1.0
WAYPOINT_INTERVAL_STEPS = int(WAYPOINT_INTERVAL_S / DT)

# Visualization style
try:
    plt.style.use('seaborn-whitegrid')
except Exception:
    # Some environments may not have seaborn styles installed; fall back gracefully
    try:
        plt.style.use('seaborn')
    except Exception:
        plt.style.use('default')

COLORS = ['C0', 'C1', 'C2', 'C3']

# -----------------------------------------------------------------------------
# Synthetic trajectory & sensors
# -----------------------------------------------------------------------------
def generate_true_trajectory(time):
    """Generate a smooth 2D path with constant altitude."""
    x = CRUISE_SPEED * time
    y = CURVE_AMP * np.sin(CURVE_FREQ * time)
    z = np.full_like(x, START_ALT)
    pos = np.vstack((x, y, z)).T
    return pos

def generate_rssi(time, jam_idx, recovery_idx=None):
    """Generate RSSI timeseries with jam at jam_idx. Optionally recover at recovery_idx."""
    rssi = RSSI_STRONG + np.random.randn(len(time)) * RSSI_NOISE_STD
    # jam period: drop to below sensitivity
    rssi[jam_idx:] = RSSI_SENSITIVITY - 6.0 + np.random.randn(len(time)-jam_idx) * 1.0
    if recovery_idx is not None and recovery_idx > jam_idx:
        rssi[recovery_idx:] = RSSI_STRONG + np.random.randn(len(time)-recovery_idx) * RSSI_NOISE_STD
    return rssi

def generate_battery_trace(initial_percent):
    """Simple linear-ish battery decline while flying; the scenario executor may alter rates."""
    # produce baseline decline but scenario will use dynamic drain during simulation
    return np.clip(initial_percent - BATTERY_DRAIN_FLY * TIME, 0.0, 100.0)

# -----------------------------------------------------------------------------
# Decision & feasibility logic
# -----------------------------------------------------------------------------
class DecisionEngine:
    """Computes feasibility (time & battery) for RTH or Retrace and chooses actions."""

    def __init__(self, fs_speed=FS_SPEED, descend_rate=DESCEND_RATE,
                 drain_fly=BATTERY_DRAIN_FLY, drain_fs=BATTERY_DRAIN_FS, drain_desc=BATTERY_DRAIN_DESCEND,
                 safety_margin=BATTERY_SAFETY_MARGIN):
        self.fs_speed = fs_speed
        self.descend_rate = descend_rate
        self.drain_fly = drain_fly
        self.drain_fs = drain_fs
        self.drain_desc = drain_desc
        self.safety = safety_margin

    def est_rth(self, start_pos, home_pos):
        horiz = np.linalg.norm((home_pos - start_pos)[:2])
        t_h = horiz / self.fs_speed
        t_d = max(0.0, start_pos[2] / self.descend_rate)
        total_t = t_h + t_d
        battery_needed = t_h * self.drain_fs + t_d * self.drain_desc
        return total_t, battery_needed

    def est_retrace(self, waypoints, start_idx):
        if start_idx >= len(waypoints) - 1:
            return 0.0, 0.0, 0.0
        length = 0.0
        for i in range(start_idx, len(waypoints) - 1):
            length += np.linalg.norm(waypoints[i+1][:2] - waypoints[i][:2])
        t_h = length / self.fs_speed
        total_t = t_h + (START_ALT / self.descend_rate)
        battery_needed = t_h * self.drain_fs + (START_ALT / self.descend_rate) * self.drain_desc
        return total_t, battery_needed, length

    def choose(self, user_pref, gps_ok, battery_pct, start_pos, home_pos, waypoints, wp_index):
        """
        user_pref: 'land', 'rth', 'retrace'
        returns dict: {'action': ..., 'reason': ..., 'est': {...}}
        possible actions:
            'land', 'rth_gps', 'rth_dr', 'retrace_gps', 'retrace_dr'
        """
        out = {'action': None, 'reason': '', 'est': {}}

        # Always compute land cost
        t_land = start_pos[2] / self.descend_rate
        b_land = t_land * self.drain_desc

        # RTH estimate
        t_rth, b_rth = self.est_rth(start_pos, home_pos)
        # Retrace estimate
        t_ret, b_ret, p_len = self.est_retrace(waypoints, wp_index)

        def feasible(b_req):
            return (battery_pct - b_req) >= self.safety

        if user_pref == 'land':
            out['action'] = 'land'
            out['reason'] = 'User preference LAND'
            out['est'] = {'time_s': t_land, 'battery_%': b_land}
            return out

        if user_pref == 'rth':
            if gps_ok:
                # GPS available: do GPS RTH if battery allows
                if feasible(b_rth):
                    out['action'] = 'rth_gps'
                    out['reason'] = 'GPS available & battery sufficient for RTH'
                else:
                    out['action'] = 'land'
                    out['reason'] = 'Battery insufficient for GPS RTH -> override LAND'
                out['est'] = {'time_s': t_rth, 'battery_%': b_rth}
                return out
            else:
                # GPS lost: try DR RTH
                if feasible(b_rth):
                    out['action'] = 'rth_dr'
                    out['reason'] = 'GPS lost -> fallback to Dead-Reckoning RTH (feasible)'
                else:
                    out['action'] = 'land'
                    out['reason'] = 'GPS lost & battery insufficient for DR RTH -> override LAND'
                out['est'] = {'time_s': t_rth, 'battery_%': b_rth}
                return out

        if user_pref == 'retrace':
            if gps_ok:
                if feasible(b_ret):
                    out['action'] = 'retrace_gps'
                    out['reason'] = 'GPS retrace feasible'
                else:
                    out['action'] = 'land'
                    out['reason'] = 'Battery insufficient for retrace -> override LAND'
                out['est'] = {'time_s': t_ret, 'battery_%': b_ret, 'path_len_m': p_len}
                return out
            else:
                if feasible(b_ret):
                    out['action'] = 'retrace_dr'
                    out['reason'] = 'GPS lost -> attempt Dead-Reckoning retrace (feasible)'
                else:
                    out['action'] = 'land'
                    out['reason'] = 'GPS lost & battery insufficient for retrace -> override LAND'
                out['est'] = {'time_s': t_ret, 'battery_%': b_ret, 'path_len_m': p_len}
                return out

        # Fallback
        out['action'] = 'land'
        out['reason'] = 'Fallback'
        out['est'] = {'time_s': t_land, 'battery_%': b_land}
        return out

# -----------------------------------------------------------------------------
# UAV state machine (simulated behaviors, power-saving, DR error)
# -----------------------------------------------------------------------------
class UAV:
    """Simple UAV model managing state transitions and execution of chosen action."""
    # States
    STATE_NORMAL = 0
    STATE_JAM = 1
    STATE_POWER_SAVE = 2
    STATE_DEADRECK = 3
    STATE_FAILSAFE_EXEC = 4
    STATE_MANUAL = 5
    STATE_LANDED = 6

    def __init__(self, true_traj, rssi, gps_ok, init_battery_pct, user_pref, name="UAV"):
        self.name = name
        self.true_traj = true_traj
        self.rssi = rssi
        self.gps_ok = gps_ok.copy()
        self.battery = generate_battery_trace(init_battery_pct)
        self.user_pref = user_pref
        self.pos = true_traj[0].copy()   # start at true initial
        self.idx = 0
        self.state = self.STATE_NORMAL
        self.powersave = False
        self.waypoints = []
        self.saved_wp_indices = []
        self.decisions = []
        self.executed_positions = []
        self.decision_engine = DecisionEngine()
        self.chosen_action = None
        self.chosen_reason = ''
        self.home = np.array([0.0, 0.0, 0.0])
        # For dead reckoning: initial estimate equals last GPS
        self.dr_pos = self.pos.copy()
        self.dr_vel = np.array([0.0, 0.0, 0.0])
        # Simulate DR drift as growing zero-mean random walk
        self.dr_drift = np.zeros(3)
        self.manual_override = False

    def save_waypoint_if_needed(self, k):
        if k % WAYPOINT_INTERVAL_STEPS == 0:
            self.waypoints.append(self.true_traj[k].copy())
            self.saved_wp_indices.append(k)

    def step(self, k):
        """Advance one time-step, record behavior and apply logic at jam event."""
        t = TIME[k]
        self.idx = k

        # Update true position
        true_pos = self.true_traj[k].copy()
        self.pos = true_pos.copy()

        # Save waypoint while GPS available
        if self.gps_ok[k]:
            self.save_waypoint_if_needed(k)
            last_gps_known = self.pos.copy()
        else:
            # GPS unavailable, last known stays
            pass

        # Monitor RSSI -> detect jam when below sensitivity
        rssi_now = self.rssi[k]
        jam_detected = rssi_now < RSSI_SENSITIVITY

        # Power saving activation on jam detection (immediate)
        if jam_detected and not self.powersave:
            self.powersave = True
            # record state change
            self.state = self.STATE_POWER_SAVE

        # If powersave active but later RSSI recovers above threshold, powersave can be reversed
        if (not jam_detected) and self.powersave and not self.manual_override:
            # For this simulation, we keep powersave until decision / manual override handling
            pass

        # At the instant of jam (first index where jam_detected and no decision yet), decide action
        if jam_detected and self.chosen_action is None:
            # Determine GPS availability at this instant
            gps_available = self.gps_ok[k]
            self.dr_pos = self.pos.copy()  # initialize DR from last known GPS
            self.dr_vel = np.array([0.0, 0.0, 0.0])
            # Ask decision engine
            dec = self.decision_engine.choose(self.user_pref, gps_available, self.battery[k],
                                              self.dr_pos, self.home, np.array(self.waypoints), 0)
            self.chosen_action = dec['action']
            self.chosen_reason = dec['reason']
            self.decisions.append((k, dec))
            # Set state to DR if gps lost
            if not gps_available and self.chosen_action in ('rth_dr', 'retrace_dr'):
                self.state = self.STATE_DEADRECK
            else:
                self.state = self.STATE_FAILSAFE_EXEC

        # Execution logic per chosen action (if any)
        if self.chosen_action is not None:
            action = self.chosen_action

            # Manual override: for scenario 4, external code may toggle self.manual_override to True
            if self.manual_override:
                self.state = self.STATE_MANUAL
                # manual pilot moves drone to an arbitrary new waypoint (simulate)
                # we'll move towards (home_x=20, home_y=-5) as manual landing target
                manual_target = np.array([20.0, -5.0, 0.0])
                self._move_towards(manual_target, FS_SPEED)
                self.executed_positions.append(self.dr_pos.copy())
                # battery drains at flight rate
                self._apply_battery_drain(mode='fs')
                # stop if landed
                if self.dr_pos[2] <= 0.0:
                    self.state = self.STATE_LANDED
                return

            if action == 'land':
                # Immediate descend in-place (use DR pos if gps lost)
                if self.gps_ok[k]:
                    loc = self.true_traj[k].copy()
                else:
                    loc = self.dr_pos.copy()
                # Hold XY, descend
                loc[2] = max(0.0, loc[2] - DESCEND_RATE * DT)
                # update DR pos if gps lost
                if not self.gps_ok[k]:
                    self.dr_pos = loc.copy()
                self.executed_positions.append(loc.copy())
                self._apply_battery_drain(mode='descend')
                if loc[2] <= 0.0:
                    self.state = self.STATE_LANDED
                return

            if action in ('rth_gps', 'rth_dr'):
                # Determine current estimate position
                if self.gps_ok[k] and action == 'rth_gps':
                    cur = self.true_traj[k].copy()
                else:
                    # DR integration: simulate simple inertial integration with drift
                    self._dead_reck_step(k)
                    cur = self.dr_pos.copy()
                # Move towards home horizontally
                vec = (self.home - cur)
                dist = np.linalg.norm(vec[:2])
                if dist > 0.5:
                    dx = (vec[:2] / (dist + 1e-9)) * FS_SPEED * DT
                    cur[0] += dx[0]
                    cur[1] += dx[1]
                else:
                    # start descent
                    cur[2] = max(0.0, cur[2] - DESCEND_RATE * DT)
                # update DR pos if needed
                if action == 'rth_dr' or not self.gps_ok[k]:
                    self.dr_pos = cur.copy()
                self.executed_positions.append(cur.copy())
                self._apply_battery_drain(mode='fs' if dist > 0.5 else 'descend')
                if cur[2] <= 0.0 and dist <= 0.5:
                    self.state = self.STATE_LANDED
                return

            if action in ('retrace_gps', 'retrace_dr'):
                # Follow stored waypoints in reverse order from last saved
                if len(self.waypoints) == 0:
                    # fallback to land
                    self.chosen_action = 'land'
                    self.chosen_reason = 'No waypoints to retrace, fallback to LAND'
                    self.state = self.STATE_FAILSAFE_EXEC
                    self.step(k)  # re-run step to handle land
                    return
                retrace_wps = list(reversed(self.waypoints))
                # Choose current position
                if self.gps_ok[k] and action == 'retrace_gps':
                    cur = self.true_traj[k].copy()
                else:
                    self._dead_reck_step(k)
                    cur = self.dr_pos.copy()
                # target waypoint is first in retrace list
                target_wp = retrace_wps[0]
                dist = np.linalg.norm((target_wp - cur)[:2])
                if dist > 0.8:
                    dir2 = (target_wp[:2] - cur[:2])
                    dir2 = dir2 / (np.linalg.norm(dir2) + 1e-9)
                    cur[0] += dir2[0] * FS_SPEED * DT
                    cur[1] += dir2[1] * FS_SPEED * DT
                else:
                    # pop waypoint
                    retrace_wps.pop(0)
                    self.waypoints = list(reversed(retrace_wps))
                # update DR pos if needed
                if action == 'retrace_dr' or not self.gps_ok[k]:
                    self.dr_pos = cur.copy()
                self.executed_positions.append(cur.copy())
                self._apply_battery_drain(mode='fs')
                # if retrace list exhausted and at ground altitude -> landed
                if len(retrace_wps) == 0 and cur[2] <= 0.0:
                    self.state = self.STATE_LANDED
                return

        # If no chosen action yet, operate normal (cruise)
        self.executed_positions.append(self.true_traj[k].copy())
        self._apply_battery_drain(mode='fly')

    def _apply_battery_drain(self, mode='fly'):
        k = self.idx
        if mode == 'fly':
            drain = BATTERY_DRAIN_FLY * DT
        elif mode == 'fs':
            drain = BATTERY_DRAIN_FS * DT
        elif mode == 'descend':
            drain = BATTERY_DRAIN_DESCEND * DT
        else:
            drain = 0.0
        # apply drain but do not go negative
        self.battery[k] = max(0.0, self.battery[k] - drain)

    def _dead_reck_step(self, k):
        """Simulate a simple dead-reckoning integration with drifting noise."""
        # Use true acceleration approx from difference of true positions (for realism)
        if k == 0:
            accel = np.zeros(3)
        else:
            vel = (self.true_traj[k] - self.true_traj[k-1]) / DT
            prev_vel = (self.true_traj[k-1] - self.true_traj[k-2]) / DT if k >= 2 else vel
            accel = (vel - prev_vel) / DT
        # integrate velocity & position, add small noise/diffusive drift
        self.dr_vel += accel * DT
        # inject small random-walk drift
        self.dr_drift += np.random.randn(3) * (0.01 * np.sqrt(DT))
        self.dr_pos += self.dr_vel * DT + self.dr_drift * DT

    def _move_towards(self, target, speed):
        """Move the dead-reckoned position towards a target at given speed (m/s) for one time-step.
        Updates self.dr_pos in-place and clamps altitude to >= 0.
        """
        dir_vec = target - self.dr_pos
        horiz_dist = np.linalg.norm(dir_vec[:2])
        if horiz_dist > 1e-6:
            step = min(speed * DT, horiz_dist)
            dir2 = dir_vec[:2] / horiz_dist
            self.dr_pos[0] += dir2[0] * step
            self.dr_pos[1] += dir2[1] * step
        # simple vertical behavior: if target altitude is lower, descend; else hold
        if self.dr_pos[2] > target[2]:
            self.dr_pos[2] = max(0.0, self.dr_pos[2] - DESCEND_RATE * DT)
        elif self.dr_pos[2] < target[2]:
            # ascend if necessary (rare in manual override), limit rate to FS_SPEED vertically for safety
            self.dr_pos[2] = min(target[2], self.dr_pos[2] + (speed * DT))

    def toggle_manual_override(self, toggle=True):
        self.manual_override = toggle

# -----------------------------------------------------------------------------
# Scenario setup & runner
# -----------------------------------------------------------------------------
def run_four_scenarios():
    true_traj = generate_true_trajectory(TIME)

    # All base RSSI traces (we will modify per scenario)
    rssi_base = generate_rssi(TIME, JAM_IDX, recovery_idx=None)

    scenarios_cfg = [
        # Scenario 1: Lucky Jam - user RTH, RSSI lost but GPS remains, battery HIGH
        {'name': "Lucky (GPS OK)", 'user_pref': 'rth', 'init_batt': 95.0,
         'jam_idx': JAM_IDX, 'gps_lost_on_jam': False, 'recovery_idx': None,
         'notes': 'RSSI lost, GPS remains; battery high -> GPS RTH'},

        # Scenario 2: Blind Jam - user RETRACE, RSSI lost and GPS lost, battery HIGH
        {'name': "Blind (GPS Lost)", 'user_pref': 'retrace', 'init_batt': 92.0,
         'jam_idx': JAM_IDX, 'gps_lost_on_jam': True, 'recovery_idx': None,
         'notes': 'RSSI & GPS lost -> Dead-Reckoning Retrace'},

        # Scenario 3: Exhausted Jam - user RTH, RSSI & GPS lost, battery CRITICALLY LOW
        {'name': "Exhausted (Low Batt)", 'user_pref': 'rth', 'init_batt': 12.0,
         'jam_idx': JAM_IDX, 'gps_lost_on_jam': True, 'recovery_idx': None,
         'notes': 'Low battery -> override to immediate LAND'},

        # Scenario 4: Recovery - user RTH, RSSI & GPS lost initially, RSSI returns mid-flight and manual override toggled
        {'name': "Recovery (Manual Override)", 'user_pref': 'rth', 'init_batt': 80.0,
         'jam_idx': JAM_IDX, 'gps_lost_on_jam': True, 'recovery_idx': RECOVERY_IDX,
         'notes': 'RSSI returns mid-maneuver -> manual override to MANUAL_CONTROL'}
    ]

    results = []
    for i, cfg in enumerate(scenarios_cfg):
        # generate RSSI specific to scenario (with optional recovery)
        rssi = generate_rssi(TIME, cfg['jam_idx'], recovery_idx=cfg['recovery_idx'])
        # GPS availability: if gps_lost_on_jam True then gps becomes False at jam idx, else True
        gps_ok = np.ones_like(rssi, dtype=bool)
        if cfg['gps_lost_on_jam']:
            gps_ok[cfg['jam_idx']:] = False
        # For scenario 4, we simulate RSSI recovery but GPS stays lost (wide-band jamming returned? choose to keep GPS lost)
        # If you want GPS to come back too, set gps_ok[recovery_idx:] = True in that scenario. For this task, GPS remains lost.
        if cfg['recovery_idx'] is not None and not cfg['gps_lost_on_jam']:
            gps_ok[cfg['recovery_idx']:] = True

        # battery baseline
        battery = generate_battery_trace(cfg['init_batt'])
        # instantiate UAV and run simulation time steps
        uav = UAV(true_traj, rssi, gps_ok, cfg['init_batt'], cfg['user_pref'], name=cfg['name'])

        # replace UAV battery array with scenario-specific baseline to allow per-step drain
        uav.battery = battery.copy()

        for k in range(N):
            # For scenario 4: simulate manual override toggle when RSSI recovers and operator toggles switch
            if cfg['name'].startswith("Recovery") and cfg['recovery_idx'] is not None and k == cfg['recovery_idx']:
                # RSSI recovered at recovery_idx; operator toggles manual override channel -> UAV responds
                # For this scenario the operator toggles override to True -> UAV should switch to MANUAL_CONTROL
                uav.toggle_manual_override(True)
            uav.step(k)
        # collect result
        results.append({
            'cfg': cfg,
            'true_traj': true_traj,
            'rssi': rssi,
            'battery': uav.battery,
            'gps_ok': gps_ok,
            'executed': np.array(uav.executed_positions),
            'waypoints': np.array(uav.waypoints),
            'decision': uav.decisions,
            'chosen_action': uav.chosen_action,
            'chosen_reason': uav.chosen_reason,
            'powersave_trace': np.array([True if (rssi[k] < RSSI_SENSITIVITY) else False for k in range(N)])
        })

    return results

# -----------------------------------------------------------------------------
# Plotting utilities
# -----------------------------------------------------------------------------
def plot_telemetry(res):
    """Generic telemetry figure for a single scenario: RSSI, battery and power-save boolean.
    Returns the matplotlib.Figure object.
    """
    rssi = res['rssi']
    batt = res['battery']
    powersave = res['powersave_trace']
    jam_idx = JAM_IDX

    # detect first recovery index after jam where RSSI rises above sensitivity
    rec_idx = None
    for k in range(jam_idx + 1, len(rssi)):
        if rssi[k] > RSSI_SENSITIVITY:
            rec_idx = k
            break

    fig, ax1 = plt.subplots(figsize=(12, 4.2))
    ax1.plot(TIME, rssi, color='tab:blue', label='RSSI (dBm)')
    ax1.axvline(TIME[jam_idx], color='red', linestyle='--', label='Jam start')
    if rec_idx is not None:
        ax1.axvline(TIME[rec_idx], color='green', linestyle='--', label='RSSI recovery')
    ax1.set_ylabel('RSSI (dBm)', color='tab:blue')
    ax1.set_ylim(RSSI_SENSITIVITY - 10, RSSI_STRONG + 10)
    ax1.tick_params(axis='y', labelcolor='tab:blue')

    ax2 = ax1.twinx()
    ax2.plot(TIME, batt, color='tab:orange', label='Battery (%)')
    ax2.set_ylabel('Battery (%)', color='tab:orange')
    ax2.set_ylim(0, 105)
    ax2.tick_params(axis='y', labelcolor='tab:orange')

    # Power saving boolean as step plot (displayed along a small y-range below RSSI)
    y_ps = np.where(powersave, RSSI_SENSITIVITY - 6.0, RSSI_SENSITIVITY + 2.0)
    ax1.step(TIME, y_ps, where='post', color='tab:purple', linewidth=2, label='Power Save (ON)')

    ax1.text(0.02, 0.9, f"Scenario: {res['cfg']['name']}", transform=ax1.transAxes, fontsize=10,
             bbox=dict(boxstyle="round", fc="wheat", alpha=0.6))
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper right')
    ax1.set_title(f"Telemetry: {res['cfg']['name']} - RSSI, Battery, Power Saving")
    plt.tight_layout()
    return fig


def plot_trajectory_single(res):
    """Plot a single scenario trajectory in its own window and return the Figure."""
    fig, ax = plt.subplots(figsize=(8, 6))
    true_traj = res['true_traj']
    exec_traj = res['executed']
    jam_pt = true_traj[JAM_IDX][:2]

    pre_jam = true_traj[:JAM_IDX+1]
    ax.plot(pre_jam[:, 0], pre_jam[:, 1], color='k', linewidth=1.1, label='True pre-jam path')
    ax.scatter(jam_pt[0], jam_pt[1], color='red', marker='x', s=80, label='Jam point')
    ax.plot(exec_traj[:, 0], exec_traj[:, 1], color='C3', linewidth=2.0, label=f"Executed: {res['chosen_action']}")
    final = exec_traj[-1]
    ax.scatter(final[0], final[1], color='C3', marker='o', s=60, label='Final position')

    cfg = res['cfg']
    gps_status = 'Lost' if cfg['gps_lost_on_jam'] else 'Available'
    txt = (f"{cfg['name']}\n"
           f"User pref: {cfg['user_pref'].upper()}\n"
           f"Battery at start: {cfg['init_batt']:.0f}%\n"
           f"GPS on jam: {gps_status}\n"
           f"Decision: {res['chosen_action']}\n"
           f"Reason: {res['chosen_reason']}")
    ax.text(0.02, 0.98, txt, transform=ax.transAxes, fontsize=9, va='top',
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85))

    ax.set_title(cfg['name'])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.axis('equal')
    ax.grid(True)
    ax.legend(fontsize='small')
    plt.tight_layout()
    return fig

def plot_trajectories_grid(results):
    """Figure 2: 2x2 grid of separated 2D Trajectories (true pre-jam, jam point, executed path)."""
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    axs = axs.flatten()

    for i, res in enumerate(results):
        ax = axs[i]
        true_traj = res['true_traj']
        exec_traj = res['executed']
        jam_pt = true_traj[JAM_IDX][:2]

        # True path BEFORE jam (to show intended route)
        pre_jam = true_traj[:JAM_IDX+1]
        ax.plot(pre_jam[:, 0], pre_jam[:, 1], color='k', linewidth=1.1, label='True pre-jam path')

        # Mark jam location
        ax.scatter(jam_pt[0], jam_pt[1], color='red', marker='x', s=80, label='Jam point')

        # Plot executed trajectory (from start to end)
        ax.plot(exec_traj[:, 0], exec_traj[:, 1], color=COLORS[i], linewidth=2.0,
                label=f"Executed: {res['chosen_action'] or 'None'}")

        # Mark final point
        final = exec_traj[-1]
        ax.scatter(final[0], final[1], color=COLORS[i], marker='o', s=60, label='Final position')

        # Text box summarizing conditions & decision
        cfg = res['cfg']
        gps_status = 'Lost' if cfg['gps_lost_on_jam'] else 'Available'
        txt = (f"{cfg['name']}\n"
               f"User pref: {cfg['user_pref'].upper()}\n"
               f"Battery at start: {cfg['init_batt']:.0f}%\n"
               f"GPS on jam: {gps_status}\n"
               f"Decision: {res['chosen_action']}\n"
               f"Reason: {res['chosen_reason']}")
        ax.text(0.02, 0.98, txt, transform=ax.transAxes, fontsize=9, va='top',
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85))

        ax.set_title(cfg['name'])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.axis('equal')
        ax.grid(True)
        ax.legend(fontsize='small')

    plt.tight_layout()
    return fig

# -----------------------------------------------------------------------------
# Main execution
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    results = run_four_scenarios()
    figs = []
    # Show telemetry for each scenario in its own window.
    for res in results:
        figs.append(plot_telemetry(res))

    # Show all four trajectories together in one 2x2 grid, one subplot per scenario.
    figs.append(plot_trajectories_grid(results))

    # Display the four telemetry windows and the trajectory overview window.
    plt.show()
