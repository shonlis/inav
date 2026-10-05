"""
UAV Smart Failsafe Simulation
- Simulates RSSI, GPS availability, battery and a straight/curved flight path
- Implements decision logic at RF jamming event to choose between user-preferred
  failsafe actions (LAND, RTH, RETRACE) and fallbacks based on GPS availability
  and battery sufficiency.
- Runs three scenarios to illustrate decision-making and fallbacks, and plots
  telemetry + spatial trajectories showing all option paths and executed paths.

Usage: run this script (Python 3.7+) with numpy and matplotlib installed.
"""

import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# Simulation configuration
# ---------------------------
SIM_SECONDS = 120.0
HZ = 10.0
DT = 1.0 / HZ
N = int(SIM_SECONDS * HZ)
TIME = np.arange(N) * DT

# Flight path
CRUISE_SPEED = 4.0  # m/s
CURVE_AMPLITUDE = 8.0
CURVE_FREQ = 0.06
START_ALT = 12.0  # meters

# RSSI / Jam
RSSI_STRONG = -40.0
RSSI_NOISE_STD = 1.0
RSSI_SENSITIVITY = -105.0
JAM_TIME = 40.0  # seconds
JAM_INDEX = int(JAM_TIME / DT)

# IMU noise parameters (used for optional DR noise)
ACC_DRIFT_STD_PER_SEC = 0.02  # m/s per sqrt(sec) drift
ANG_DRIFT_STD_PER_SEC = 0.002  # rad per sqrt(sec)

# Battery model (percent)
BATTERY_DRAIN_PER_SEC = 0.25  # percent battery consumed per second while flying
BATTERY_DESCEND_PER_SEC = 0.08  # percent per second while descending slowly
BATTERY_SAFETY_MARGIN = 3.0  # reserve percent required

# Failsafe motion parameters
FS_CRUISE_SPEED = 3.0  # m/s horizontal during failsafe
DESCEND_RATE = 0.6  # m/s

# Waypoint saving
WAYPOINT_INTERVAL_SEC = 1.0
WAYPOINT_INTERVAL_STEPS = int(WAYPOINT_INTERVAL_SEC / DT)

np.random.seed(0)

# ---------------------------
# Utility functions
# ---------------------------

def generate_true_trajectory(time):
    """Generate a smooth 2D trajectory with altitude."""
    x = CRUISE_SPEED * time
    y = CURVE_AMPLITUDE * np.sin(CURVE_FREQ * time)
    z = np.full_like(x, START_ALT)
    pos = np.vstack((x, y, z)).T
    return pos


def generate_rssi(time, jam_idx):
    rssi = RSSI_STRONG + np.random.randn(len(time)) * RSSI_NOISE_STD
    # apply jam dropout (RSSI drops) from jam_idx onward
    rssi[jam_idx:] = RSSI_SENSITIVITY - 5.0 + np.random.randn(len(time) - jam_idx) * 1.0
    return rssi


def generate_battery_baseline(time, start_percent=100.0, end_percent=20.0):
    # Linear baseline declining (not used for scenario-specific initial battery)
    return np.linspace(start_percent, end_percent, len(time))

# ---------------------------
# Decision logic
# ---------------------------

class FailsafeDecision:
    """Encapsulate decision computations: battery feasibility estimates and chosen action."""

    def __init__(self, fs_speed=FS_CRUISE_SPEED, descend_rate=DESCEND_RATE,
                 battery_drain_flying=BATTERY_DRAIN_PER_SEC,
                 battery_drain_descend=BATTERY_DESCEND_PER_SEC,
                 safety_margin=BATTERY_SAFETY_MARGIN):
        self.fs_speed = fs_speed
        self.descend_rate = descend_rate
        self.battery_drain_flying = battery_drain_flying
        self.battery_drain_descend = battery_drain_descend
        self.safety_margin = safety_margin

    def estimate_time_and_battery_for_rth(self, start_pos, home_pos, current_alt, battery_remain):
        """Estimate time and battery required to RTH and land from start_pos.
        Returns (time_s, battery_needed_percent)
        """
        horiz_dist = np.linalg.norm((home_pos - start_pos)[:2])
        t_horizontal = horiz_dist / self.fs_speed
        t_descend = max(0.0, current_alt / self.descend_rate)
        t_total = t_horizontal + t_descend
        # battery consumption approx as seconds * rate
        battery_needed = t_horizontal * self.battery_drain_flying + t_descend * self.battery_drain_descend
        return t_total, battery_needed

    def estimate_time_and_battery_for_retrace(self, waypoints, start_idx, current_alt):
        """Estimate time and battery for retracing waypoints from start_idx (index into waypoints list)
        Assumes waypoints are in world coordinates, retrace travels sequentially from start_idx to end.
        Returns (time_s, battery_needed_percent, path_length)
        """
        if start_idx >= len(waypoints) - 1:
            return 0.0, 0.0, 0.0
        # compute path length along remaining waypoints
        total_len = 0.0
        for i in range(start_idx, len(waypoints) - 1):
            p0 = waypoints[i][:2]
            p1 = waypoints[i + 1][:2]
            total_len += np.linalg.norm(p1 - p0)
        # time to traverse
        t_horizontal = total_len / self.fs_speed
        t_descend = max(0.0, current_alt / self.descend_rate)
        t_total = t_horizontal + t_descend
        battery_needed = t_horizontal * self.battery_drain_flying + t_descend * self.battery_drain_descend
        return t_total, battery_needed, total_len

    def choose_action(self, user_choice, gps_available, battery_remain, start_pos, home_pos, waypoints, wp_index):
        """Decide best feasible action and return chosen action plus metadata.
        Allowed actions: 'land', 'rth_gps', 'rth_dr', 'retrace_gps', 'retrace_dr'
        The user_choice is one of 'land', 'rth', 'retrace'
        """
        result = {
            'chosen': None,
            'reason': '',
            'plan': None
        }

        # Evaluate options
        # 1) LAND is always feasible (quick descend). Estimate battery
        t_land = start_pos[2] / self.descend_rate
        b_land = t_land * self.battery_drain_descend

        # 2) RTH via GPS if gps_available, else via dead-reckoning
        t_rth, b_rth = self.estimate_time_and_battery_for_rth(start_pos, home_pos, start_pos[2], battery_remain)

        # 3) Retrace
        t_retrace, b_retrace, path_len = self.estimate_time_and_battery_for_retrace(waypoints, wp_index, start_pos[2])

        # Helper check function
        def feasible(b_needed, battery_remain):
            return (battery_remain - b_needed) >= self.safety_margin

        # Decision flow
        if user_choice == 'land':
            # user wants land; accept unless some other reason
            if feasible(b_land, battery_remain):
                result['chosen'] = 'land'
                result['reason'] = 'User selected LAND (feasible)'
            else:
                # if even land not feasible (extremely low battery), choose immediate land anyway
                result['chosen'] = 'land'
                result['reason'] = 'Battery critical; forced LAND'
            result['plan'] = {'est_time_s': t_land, 'est_battery_pct': b_land}
            return result

        if user_choice == 'rth':
            # prefer GPS RTH if available
            if gps_available:
                if feasible(b_rth, battery_remain):
                    result['chosen'] = 'rth_gps'
                    result['reason'] = 'User selected RTH and GPS available (feasible)'
                else:
                    # battery insufficient for GPS RTH -> land
                    result['chosen'] = 'land'
                    result['reason'] = 'Battery insufficient for RTH; override to LAND'
                result['plan'] = {'est_time_s': t_rth, 'est_battery_pct': b_rth}
                return result
            else:
                # GPS not available -> fallback to DR RTH: still possible but less accurate
                if feasible(b_rth, battery_remain):
                    result['chosen'] = 'rth_dr'
                    result['reason'] = 'GPS lost; fallback to Dead-Reckoning RTH (feasible)'
                else:
                    result['chosen'] = 'land'
                    result['reason'] = 'GPS lost and battery insufficient for DR RTH; override to LAND'
                result['plan'] = {'est_time_s': t_rth, 'est_battery_pct': b_rth}
                return result

        if user_choice == 'retrace':
            # If enough battery for retrace using GPS, do it. If gps lost, retrace via DR if possible.
            if gps_available:
                if feasible(b_retrace, battery_remain):
                    result['chosen'] = 'retrace_gps'
                    result['reason'] = 'User selected RETRACE and GPS available (feasible)'
                else:
                    result['chosen'] = 'land'
                    result['reason'] = 'Battery insufficient for RETRACE; override to LAND'
                result['plan'] = {'est_time_s': t_retrace, 'est_battery_pct': b_retrace, 'path_len': path_len}
                return result
            else:
                # gps lost -> retrace via DR
                if feasible(b_retrace, battery_remain):
                    result['chosen'] = 'retrace_dr'
                    result['reason'] = 'GPS lost; attempt Dead-Reckoning RETRACE (feasible)'
                else:
                    result['chosen'] = 'land'
                    result['reason'] = 'GPS lost and battery insufficient for RETRACE; override to LAND'
                result['plan'] = {'est_time_s': t_retrace, 'est_battery_pct': b_retrace, 'path_len': path_len}
                return result

        # fallback default
        result['chosen'] = 'land'
        result['reason'] = 'Default fallback'
        result['plan'] = {'est_time_s': t_land, 'est_battery_pct': b_land}
        return result

# ---------------------------
# Execution simulation
# ---------------------------

def execute_action_from_index(action, start_idx, start_pos, true_traj, waypoints, gps_available_at_execution,
                              desc_rate=DESCEND_RATE, fs_speed=FS_CRUISE_SPEED):
    """Simulate execution starting at time index start_idx.
    Returns executed_positions shaped (N - start_idx, 3) showing the path from start_idx to end.
    Also returns intended option paths for plotting (land path, rth path, retrace path).
    """
    executed = []
    Nrem = len(true_traj) - start_idx
    pos = start_pos.copy()
    executed.append(pos.copy())

    # For planning visuals: compute option paths (from exact last-known start_pos)
    home = np.array([0.0, 0.0, 0.0])
    # Land path: vertical descent only
    land_path = []
    tmppos = pos.copy()
    while tmppos[2] > 0 and len(land_path) < 10000:
        tmppos = tmppos.copy()
        tmppos[2] = max(0.0, tmppos[2] - desc_rate * DT)
        land_path.append(tmppos.copy())

    # RTH straight-line path (ideal) from start_pos to home (horizontal) then descend
    rth_path = []
    # sample along straight line
    horiz_vec = home - pos
    horiz_vec_z0 = np.array([horiz_vec[0], horiz_vec[1], 0.0])
    horiz_dist = np.linalg.norm(horiz_vec_z0[:2])
    steps_horiz = int(np.ceil(horiz_dist / (fs_speed * DT))) if horiz_dist > 1e-6 else 0
    for s in range(steps_horiz):
        frac = (s + 1) / max(1, steps_horiz)
        p = pos + np.hstack((horiz_vec_z0[:2] * frac, 0.0))
        rth_path.append(p.copy())
    # then descend at pos = home
    descpos = home.copy()
    while descpos[2] > 0 and len(rth_path) < 20000:
        descpos = descpos.copy()
        descpos[2] = max(0.0, descpos[2] - desc_rate * DT)
        rth_path.append(descpos.copy())

    # Retrace path: follow reversed waypoints then land
    retrace_path = []
    if len(waypoints) > 0:
        retrace_waypoints = list(reversed(waypoints))
        # find nearest waypoint index to start_pos in retrace list
        # we'll simulate following from first retrace wp
        current = pos.copy()
        for wp in retrace_waypoints:
            wp2 = wp.copy()
            # travel towards wp in small steps
            while np.linalg.norm((wp2 - current)[:2]) > fs_speed * DT * 1.05:
                dir2 = (wp2 - current)[:2]
                dir2 = dir2 / (np.linalg.norm(dir2) + 1e-9)
                current[0] += dir2[0] * fs_speed * DT
                current[1] += dir2[1] * fs_speed * DT
                current[2] = max(current[2], wp2[2])  # maintain altitude until waypoint
                retrace_path.append(current.copy())
            # snap to waypoint
            current = wp2.copy()
            retrace_path.append(current.copy())
        # then descend
        while current[2] > 0:
            current = current.copy()
            current[2] = max(0.0, current[2] - desc_rate * DT)
            retrace_path.append(current.copy())

    # Now simulate the executed trajectory according to chosen action
    if action == 'land':
        # hold XY and descend
        for step in range(Nrem - 1):
            pos[2] = max(0.0, pos[2] - desc_rate * DT)
            executed.append(pos.copy())
            if pos[2] <= 0.0:
                # landed, hold
                for _ in range(step + 1, Nrem - 1):
                    executed.append(pos.copy())
                break

    elif action in ('rth_gps', 'rth_dr'):
        # Move horizontally towards home then descend
        home = np.array([0.0, 0.0, 0.0])
        for step in range(Nrem - 1):
            vec = home - pos
            dist = np.linalg.norm(vec[:2])
            if dist > 0.5:
                dir2 = vec[:2] / (dist + 1e-9)
                pos[0] += dir2[0] * fs_speed * DT
                pos[1] += dir2[1] * fs_speed * DT
                # maintain altitude until near home
            else:
                # start descent
                pos[2] = max(0.0, pos[2] - desc_rate * DT)
            executed.append(pos.copy())
            if pos[2] <= 0.0 and dist <= 0.5:
                # landed at home
                for _ in range(step + 1, Nrem - 1):
                    executed.append(pos.copy())
                break
        # add drift if action is rth_dr (dead-reckoning) to simulate estimation error
        if action == 'rth_dr':
            executed = np.array(executed)
            # add gradual lateral drift noise growing with sqrt(time)
            tvec = np.arange(len(executed)) * DT
            drift_x = np.cumsum(np.random.randn(len(executed)) * ACC_DRIFT_STD_PER_SEC * np.sqrt(DT))
            drift_y = np.cumsum(np.random.randn(len(executed)) * ACC_DRIFT_STD_PER_SEC * np.sqrt(DT))
            executed[:, 0] += drift_x
            executed[:, 1] += drift_y
            executed = list(executed)

    elif action in ('retrace_gps', 'retrace_dr'):
        # Follow retrace_path until exhausted then land
        if len(retrace_path) == 0:
            # nothing to retrace -> land
            for step in range(Nrem - 1):
                pos[2] = max(0.0, pos[2] - desc_rate * DT)
                executed.append(pos.copy())
                if pos[2] <= 0.0:
                    for _ in range(step + 1, Nrem - 1):
                        executed.append(pos.copy())
                    break
        else:
            # step through retrace_path
            idx = 0
            for step in range(Nrem - 1):
                if idx < len(retrace_path):
                    pos = retrace_path[idx].copy()
                    idx += 1
                else:
                    # landed or hold
                    pos[2] = max(0.0, pos[2] - desc_rate * DT)
                executed.append(pos.copy())
            # add drift to simulate DR error when gps not available
            if action == 'retrace_dr':
                executed = np.array(executed)
                drift_x = np.cumsum(np.random.randn(len(executed)) * ACC_DRIFT_STD_PER_SEC * np.sqrt(DT))
                drift_y = np.cumsum(np.random.randn(len(executed)) * ACC_DRIFT_STD_PER_SEC * np.sqrt(DT))
                executed[:, 0] += drift_x
                executed[:, 1] += drift_y
                executed = list(executed)

    else:
        # unknown action -> hold
        for _ in range(Nrem - 1):
            executed.append(pos.copy())

    executed = np.array(executed)
    return executed, np.array(land_path), np.array(rth_path), np.array(retrace_path)

# ---------------------------
# Scenario runner
# ---------------------------

def run_scenarios():
    # Common true trajectory
    true_traj = generate_true_trajectory(TIME)
    rssi = generate_rssi(TIME, JAM_INDEX)

    scenarios = []

    # Scenario A: Adequate Battery, GPS Lost at jam, user prefers RTH -> fallback to DR RTH
    scenarios.append({
        'name': 'A: Pref RTH, GPS lost, Adequate Battery',
        'user_choice': 'rth',
        'initial_battery': 80.0,
        'gps_lost_on_jam': True
    })

    # Scenario B: Low Battery, GPS Lost at jam, user prefers RETRACE -> override to LAND
    scenarios.append({
        'name': 'B: Pref RETRACE, GPS lost, Low Battery',
        'user_choice': 'retrace',
        'initial_battery': 10.0,
        'gps_lost_on_jam': True
    })

    # Scenario C: Adequate Battery, GPS REMAINS available (jam affects comm only), user prefers RETRACE -> succeed
    scenarios.append({
        'name': 'C: Pref RETRACE, GPS available, Adequate Battery',
        'user_choice': 'retrace',
        'initial_battery': 85.0,
        'gps_lost_on_jam': False
    })

    results = []
    decider = FailsafeDecision()

    for scen in scenarios:
        gps_ok = np.ones_like(rssi, dtype=bool)
        if scen['gps_lost_on_jam']:
            gps_ok[JAM_INDEX:] = False
        else:
            gps_ok[:] = True

        # Battery trace for this scenario starting at initial_battery and decaying by drain per second
        battery = np.clip(scen['initial_battery'] - BATTERY_DRAIN_PER_SEC * TIME, 0.0, 100.0)

        # Save waypoints while GPS available up to jam (simulate perfect GPS waypoints)
        waypoints = []
        for k in range(0, JAM_INDEX + 1):
            if k % WAYPOINT_INTERVAL_STEPS == 0 and gps_ok[k]:
                waypoints.append(true_traj[k].copy())

        # At jam time, decide
        start_pos = true_traj[JAM_INDEX].copy()
        battery_at_jam = battery[JAM_INDEX]
        gps_available_at_jam = gps_ok[JAM_INDEX]

        decision = decider.choose_action(user_choice=scen['user_choice'],
                                         gps_available=gps_available_at_jam,
                                         battery_remain=battery_at_jam,
                                         start_pos=start_pos,
                                         home_pos=np.array([0.0, 0.0, 0.0]),
                                         waypoints=np.array(waypoints),
                                         wp_index=0)

        chosen = decision['chosen']

        # Execute chosen action from jam index onward
        executed, land_path, rth_path, retrace_path = execute_action_from_index(chosen, JAM_INDEX, start_pos,
                                                                                 true_traj, waypoints,
                                                                                 gps_available_at_jam)

        # Build full trajectory (true before jam, executed after)
        full_exec = np.vstack((true_traj[:JAM_INDEX], executed))

        results.append({
            'scenario': scen,
            'rssi': rssi,
            'battery': battery,
            'gps_ok': gps_ok,
            'waypoints': np.array(waypoints),
            'decision': decision,
            'chosen_action': chosen,
            'executed_traj': full_exec,
            'land_path': land_path,
            'rth_path': rth_path,
            'retrace_path': retrace_path,
            'true_traj': true_traj
        })

    return results

# ---------------------------
# Plotting
# ---------------------------

def plot_all(results):
    fig = plt.figure(figsize=(14, 10))

    # Top: telemetry (RSSI, battery for each scenario, GPS status per scenario)
    ax1 = plt.subplot2grid((3, 2), (0, 0), colspan=2)
    t = TIME
    # Plot RSSI
    ax1.plot(t, results[0]['rssi'], color='C0', label='RSSI (dBm)')
    ax1.axvline(JAM_TIME, color='red', linestyle='--', label='Jamming Event')
    ax1.set_ylabel('RSSI (dBm)', color='C0')
    ax1.tick_params(axis='y', labelcolor='C0')
    ax1.grid(True)

    # Plot battery traces for each scenario on right axis
    ax1b = ax1.twinx()
    colors = ['C1', 'C2', 'C3']
    for i, res in enumerate(results):
        ax1b.plot(t, res['battery'], color=colors[i], label=f"Battery ({res['scenario']['name']})")
    ax1b.set_ylabel('Battery (%)')
    ax1b.set_ylim(0, 105)

    # Plot GPS availability as step markers per scenario (offset vertically)
    ybase = -115
    for i, res in enumerate(results):
        gps_bool = res['gps_ok']
        yvals = np.where(gps_bool, ybase + i * 2, ybase - 2 + i * 2)
        ax1.plot(t, yvals, drawstyle='steps-post', color=colors[i], alpha=0.5,
                 label=f"GPS OK ({res['scenario']['name']})")
    ax1.set_ylim(RSSI_SENSITIVITY - 10, RSSI_STRONG + 10)
    ax1.legend(loc='upper right')
    ax1.set_title('Telemetry: RSSI, Battery, GPS status (vertical line = jamming event)')

    # Bottom-left: trajectories map
    ax2 = plt.subplot2grid((3, 2), (1, 0), rowspan=2)
    # Plot true path
    true = results[0]['true_traj']
    ax2.plot(true[:, 0], true[:, 1], color='k', linewidth=1.2, label='True Path')
    # Mark jam point
    jam_xy = true[JAM_INDEX, :2]
    ax2.scatter(jam_xy[0], jam_xy[1], color='red', marker='x', s=80, label='Jam Point')

    # Plot planned option paths (from first scenario start_pos)
    # Use first scenario's computed option paths as representative visuals
    for i, res in enumerate(results):
        # plot available option plans (dashed)
        if res['land_path'].size > 0:
            ax2.plot(res['land_path'][:, 0], res['land_path'][:, 1], '--', color=colors[i], alpha=0.6,
                     label=f"Land path ({res['scenario']['name']})")
        if res['rth_path'].size > 0:
            ax2.plot(res['rth_path'][:, 0], res['rth_path'][:, 1], ':', color=colors[i], alpha=0.6,
                     label=f"RTH plan ({res['scenario']['name']})")
        if res['retrace_path'].size > 0:
            ax2.plot(res['retrace_path'][:, 0], res['retrace_path'][:, 1], '-.', color=colors[i], alpha=0.6,
                     label=f"Retrace plan ({res['scenario']['name']})")

    # Plot executed trajectories for each scenario
    linestyles = ['-', '--', '-.']
    for i, res in enumerate(results):
        exec_traj = res['executed_traj']
        ax2.plot(exec_traj[:, 0], exec_traj[:, 1], linestyle=linestyles[i], color=colors[i], linewidth=2.0,
                 label=f"Executed ({res['scenario']['name']}) - {res['chosen_action']}")
        # mark landing point (last point)
        lp = exec_traj[-1]
        ax2.scatter(lp[0], lp[1], color=colors[i], marker='o', s=50)

    ax2.set_title('2D Trajectories (XY) - True, Planned options (dashed), and Executed paths')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.legend(fontsize='small')
    ax2.axis('equal')
    ax2.grid(True)

    # Right top: summary table of decisions
    ax3 = plt.subplot2grid((3, 2), (1, 1))
    ax3.axis('off')
    lines = []
    for res in results:
        s = res['scenario']
        dec = res['decision']
        lines.append(f"Scenario: {s['name']}")
        lines.append(f"  User choice: {s['user_choice'].upper()}")
        lines.append(f"  GPS lost on jam: {s['gps_lost_on_jam']}")
        lines.append(f"  Battery at jam: {res['battery'][JAM_INDEX]:.1f}%")
        lines.append(f"  Decision: {dec['chosen']}  ({dec['reason']})")
        lines.append("")
    ax3.text(0.01, 0.99, '\n'.join(lines), va='top', fontsize=10, family='monospace')

    # Right bottom: show waypoints count and example waypoints
    ax4 = plt.subplot2grid((3, 2), (2, 1))
    ax4.axis('off')
    lines2 = []
    for res in results:
        lines2.append(f"{res['scenario']['name']}: waypoints saved = {len(res['waypoints'])}")
    ax4.text(0.01, 0.99, '\n'.join(lines2), va='top', fontsize=10)

    plt.tight_layout()
    plt.show()

# ---------------------------
# Main
# ---------------------------
if __name__ == '__main__':
    results = run_scenarios()
    plot_all(results)

