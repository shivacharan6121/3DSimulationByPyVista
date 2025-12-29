import numpy as np
import pandas as pd

# ============================================================
# SIMULATION PARAMETERS
# ============================================================
dt = 0.1                 # 2.5 ms
tmax = 211.0
times = np.arange(0, tmax, dt)
n = len(times)

# Target & missile parameters (same logic as your animation)
Straight_time = 25
curve_time = 25
Straight_time2 = 25

targ_vel = 750.0            # m/s
miss_vel = 800.0            # m/s

turn_angle = -np.pi * 4/3
yz_angle = -np.pi / 12
climb_rate_curve = -0.001

missile_start_loc = np.array([13000.0, 12000.0, 0.0])
aircraft_start_loc = np.array([0.0, 0.0, 12000.0])

missile_launch_time = 0.0
kill_dist = 50.0

# ============================================================
# TARGET TRAJECTORY FUNCTION (UNCHANGED LOGIC)
# ============================================================
radius = (targ_vel * curve_time) / turn_angle

curve_initialized = False
straight2_initialized = False
curve_start = np.zeros(3)
straight2_start = np.zeros(3)
center = np.zeros(3)

def target_location(t, prev_states):
    global curve_initialized, straight2_initialized
    global curve_start, straight2_start, center

    if 0 <= t <= Straight_time:
        return aircraft_start_loc + np.array([targ_vel * t, 0, 0])

    elif Straight_time < t <= Straight_time + curve_time:
        tc = t - Straight_time

        if not curve_initialized:
            curve_start = prev_states[-1]
            center[0] = curve_start[0]
            center[1] = curve_start[1] + radius * np.cos(yz_angle)
            center[2] = curve_start[2] + radius * np.sin(yz_angle)
            curve_initialized = True

        angle = tc * turn_angle / curve_time
        arc_angle = -np.pi / 2 + angle

        x = center[0] + radius * np.cos(arc_angle)
        y = center[1] + radius * np.sin(arc_angle) * np.cos(yz_angle)
        z = center[2] + radius * np.sin(arc_angle) * np.sin(yz_angle)

        climb = targ_vel**2 * (1 - np.cos(np.pi * tc / curve_time)) * climb_rate_curve
        y += np.cos(yz_angle + np.pi/2) * climb
        z += np.sin(yz_angle + np.pi/2) * climb

        return np.array([x, y, z])

    elif Straight_time + curve_time < t <= Straight_time + curve_time + Straight_time2:
        if not straight2_initialized:
            straight2_start = prev_states[-1]
            straight2_initialized = True

        ts = t - (Straight_time + curve_time)
        dx = np.cos(turn_angle)
        dy = np.sin(turn_angle) * np.cos(yz_angle)
        dz = np.sin(turn_angle) * np.sin(yz_angle)

        return straight2_start + targ_vel * ts * np.array([dx, dy, dz])

    else:
        dx = np.cos(turn_angle)
        dy = np.sin(turn_angle) * np.cos(yz_angle)
        dz = np.sin(turn_angle) * np.sin(yz_angle)
        return straight2_start + targ_vel * Straight_time2 * np.array([dx, dy, dz])

# ============================================================
# GENERATE TARGET STATES
# ============================================================
target_pos = np.zeros((n, 3))
target_vel = np.zeros((n, 3))

for i, t in enumerate(times):
    target_pos[i] = target_location(t, target_pos[:i]) if i > 0 else aircraft_start_loc

target_vel[1:] = np.diff(target_pos, axis=0) / dt
target_vel[0] = target_vel[1]

# ============================================================
# GENERATE MISSILE STATES (PURE PURSUIT)
# ============================================================
missile_pos = np.zeros((n, 3))
missile_vel = np.zeros((n, 3))

missile_pos[0] = missile_start_loc
missile_launched = False
intercepted = False

for i in range(1, n):
    t = times[i]

    if t >= missile_launch_time:
        missile_launched = True

    if missile_launched and not intercepted:
        direction = target_pos[i] - missile_pos[i-1]
        dist = np.linalg.norm(direction)

        if dist < kill_dist:
            intercepted = True
            missile_pos[i] = missile_pos[i-1]
            missile_vel[i] = np.zeros(3)
        else:
            v = (direction / dist) * miss_vel
            missile_vel[i] = v
            missile_pos[i] = missile_pos[i-1] + v * dt
    else:
        missile_pos[i] = missile_pos[i-1]
        missile_vel[i] = np.zeros(3)

missile_vel[0] = missile_vel[1]

# ============================================================
# STORE TO CSV (MISSILE FIRST, THEN TARGET)
# ============================================================
data = np.column_stack([
    times,
    missile_pos, missile_vel,
    target_pos, target_vel
])

columns = [
    "time",
    "mx", "my", "mz", "mvx", "mvy", "mvz",
    "tx", "ty", "tz", "tvx", "tvy", "tvz"
]

df = pd.DataFrame(data, columns=columns)
df.to_csv("missile_target_simulation_211s_2p5ms.csv", index=False)

print("CSV generated successfully:")
print("missile_target_simulation_211s_2p5ms.csv")
print(f"Rows: {len(df)}")
