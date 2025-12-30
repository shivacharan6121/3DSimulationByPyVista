import numpy as np
import pandas as pd
import trimesh
import pyvista as pv
import tkinter as tk
from tkinter import filedialog
import time

# ============================================================
# FILE SELECTION
# ============================================================
def select_csv_file():
    root = tk.Tk()
    root.withdraw()
    return filedialog.askopenfilename(
        title="Select Trajectory CSV File",
        filetypes=[("CSV Files", "*.csv")]
    )

# ============================================================
# LOAD CSV
# ============================================================
file_path = select_csv_file()
if not file_path:
    raise RuntimeError("No CSV selected")

data = pd.read_csv(file_path)

times = data["time"].values
missile_pos = data[["mx","my","mz"]].values
missile_vel = data[["mvx","mvy","mvz"]].values
target_pos  = data[["tx","ty","tz"]].values
target_vel  = data[["tvx","tvy","tvz"]].values

# ============================================================
# HIT DETECTION
# ============================================================
KILL_DIST = 35.0
ranges = np.linalg.norm(target_pos - missile_pos, axis=1)
hit_idx = np.where(ranges <= KILL_DIST)[0]
hit_idx = hit_idx[0] if len(hit_idx) else len(times)-1

# ============================================================
# ROTATION FROM VELOCITY
# ============================================================
def rotation_from_velocity(v):
    n = np.linalg.norm(v)
    if n < 1e-6:
        return np.eye(3)

    v = v / n
    yaw = np.arctan2(v[1], v[0])
    pitch = np.arctan2(v[2], np.sqrt(v[0]**2 + v[1]**2))

    Rz = np.array([[ np.cos(yaw),-np.sin(yaw),0],
                   [ np.sin(yaw), np.cos(yaw),0],
                   [0,0,1]])

    Ry = np.array([[ np.cos(-pitch),0,np.sin(-pitch)],
                   [0,1,0],
                   [-np.sin(-pitch),0,np.cos(-pitch)]])

    return Rz @ Ry



# ============================================================
# HEADING ANGLE
# ============================================================
def heading_deg(v, mp, tp):
    los = tp - mp
    nv = np.linalg.norm(v)
    nl = np.linalg.norm(los)
    if nv < 1e-6 or nl < 1e-6:
        return 0.0
    v = v / nv
    los = los / nl
    return np.degrees(np.arccos(np.clip(np.dot(v, los), -1, 1)))




# ============================================================
# LOAD GLTF
# ============================================================
def load_gltf(path, scale, base_rot):
    scene = trimesh.load(path, force="scene")
    mesh = trimesh.util.concatenate(scene.geometry.values())
    mesh.vertices -= mesh.vertices.mean(axis=0)
    mesh.apply_scale(scale)
    mesh.vertices = mesh.vertices @ base_rot.T
    return pv.PolyData(mesh.vertices, np.hstack(
        (np.full((len(mesh.faces),1),3), mesh.faces)
    ))

MISSILE_ROT = np.array([[0,0,1],[0,1,0],[-1,0,0]])
AIRCRAFT_ROT = np.array([[0,-1,0],[1,0,0],[0,0,1]])

missile_mesh = load_gltf("./missile/scene.gltf", 2000, MISSILE_ROT)
target_mesh  = load_gltf("./r1/scene.gltf", 800, AIRCRAFT_ROT)



# ============================================================
# PYVISTA SCENE
# ============================================================
plotter = pv.Plotter(window_size=(1600,900))
plotter.set_background("black")
plotter.add_text("MISSILE Vs TARGET SIMULATION", position="upper_edge", font_size=16, color='white')
plotter.show_axes()

# ============================================================
# MODEL ACTORS
# ============================================================
missile_actor = plotter.add_mesh(
    missile_mesh.copy(),
    color="cyan",
    smooth_shading=True
)

target_actor = plotter.add_mesh(
    target_mesh.copy(),
    color="red",
    opacity=0.6,
    smooth_shading=True
)

# ============================================================
# ATTACHED PATH LINES (MODEL-FOLLOWING)
# ============================================================
missile_path = pv.PolyData(missile_pos[:1])
missile_path.lines = np.array([1, 0])
missile_path_actor = plotter.add_mesh(missile_path, color="cyan", line_width=2, opacity =0.2)

target_path = pv.PolyData(target_pos[:1])
target_path.lines = np.array([1, 0])
target_path_actor = plotter.add_mesh(target_path, color="red", line_width=2, opacity=0.3)

# HUD
info = plotter.add_text("", position="upper_left", font_size=8, color="green")

info.prop.opacity = 1.0

# ============================================================
# RECORD
# ============================================================
plotter.open_movie("missile_vs_target.mp4", framerate=50)
plotter.show(interactive_update=True, auto_close=False)

# ============================================================
# ANIMATION LOOP
# ============================================================
for i in range(hit_idx + 50):
    idx = min(i, hit_idx)

    # ---- UPDATE PATH LINES (ATTACHED) ----
    missile_path.points = missile_pos[:idx+1]
    missile_path.lines = np.hstack([[idx+1], np.arange(idx+1)])

    target_path.points = target_pos[:idx+1]
    target_path.lines = np.hstack([[idx+1], np.arange(idx+1)])

    # ---- UPDATE MODELS ----
    missile_actor.mapper.dataset.points = (
        missile_mesh.points @ rotation_from_velocity(missile_vel[idx]).T
        + missile_pos[idx]
    )

    target_actor.mapper.dataset.points = (
        target_mesh.points @ rotation_from_velocity(target_vel[idx]).T
        + target_pos[idx]
    )
    
    sep = np.linalg.norm(target_pos[idx] - missile_pos[idx])
    hdg = heading_deg(missile_vel[idx], missile_pos[idx], target_pos[idx])

    info.set_text('upper_left', (
        "MISSILE\n"
        f" X:{missile_pos[idx,0]:8.1f} m\n"
        f" Y:{missile_pos[idx,1]:8.1f} m\n"
        f" Z:{missile_pos[idx,2]:8.1f} m\n\n"
        "TARGET\n"
        f" X:{target_pos[idx,0]:8.1f} m\n"
        f" Y:{target_pos[idx,1]:8.1f} m\n"
        f" Z:{target_pos[idx,2]:8.1f} m\n\n"
        f"SEP   : {sep:8.2f} m\n"
        f"TIME  : {times[idx]:.2f} s\n"
        f"HDG   : {hdg:.2f} deg\n"
        f"STAT  : {'HIT' if idx >= hit_idx else 'TRACK'}"
    ))


    plotter.render()
    plotter.write_frame()
    time.sleep(0.02)

plotter.close()
