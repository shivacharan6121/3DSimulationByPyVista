# Missile vs Target 3D Trajectory Simulation (PyVista)

A **file-driven 3D missile–target engagement visualizer** built with **Python + PyVista**.
The program reads a CSV trajectory file, animates missile and target motion using **GLTF 3D models**, computes **heading angle**, **separation distance**, detects **hit condition**, and records the full simulation as an **MP4 video**.

---

## 🚀 Features

* 📂 CSV-based trajectory input (no hard‑coded paths)
* 🎯 Missile–target **hit detection** using configurable kill distance
* 🧭 Real-time **heading (LOS) angle** computation
* 🛰 Velocity-aligned **3D GLTF models** (missile & aircraft)
* ✈️ Animated **motion trails** for missile and target
* 🎥 High‑quality **MP4 recording** (off‑screen safe)
* 🖥 Interactive PyVista window with HUD overlay

---

## 📁 Project Structure

```
project_root/
│
├── pyvista.py                     # Simulation script
├── missile/
│   └── scene.gltf              # Missile 3D model
├── r1/
│   └── scene.gltf              # Target aircraft 3D model
├── data/
│   └── trajectory.csv          # Input CSV file
└── missile_vs_target.mp4       # Output video (generated)
```

---

## 📊 CSV File Format

The CSV **must contain** the following columns:

```text
time,
mx,my,mz,
mvx,mvy,mvz,
tx,ty,tz,
tvx,tvy,tvz
```

### Meaning

| Column      | Description               |
| ----------- | ------------------------- |
| time        | Simulation time (seconds) |
| mx,my,mz    | Missile position (meters) |
| mvx,mvy,mvz | Missile velocity (m/s)    |
| tx,ty,tz    | Target position (meters)  |
| tvx,tvy,tvz | Target velocity (m/s)     |

---

## 🧠 Core Logic

### Hit Detection

```python
KILL_DIST = 35.0  # meters
```

The simulation is marked **HIT** when:

```
|target_pos - missile_pos| ≤ KILL_DIST
```

---

### Velocity-Based Model Orientation

* Models automatically **rotate to align with velocity vectors**
* Uses yaw–pitch rotation derived from velocity direction
* Prevents gimbal lock and zero-velocity crashes

---

### Heading (LOS) Angle

The heading angle is computed between:

* Missile velocity vector
* Line-of-sight (missile → target)

Displayed live in the HUD (degrees).

---

## 🖥 HUD Overlay

Displayed in the top-left corner:

* Missile XYZ position
* Target XYZ position
* Separation distance
* Simulation time
* Heading angle
* Status: `TRACK` or `HIT`

---

## 🎥 Video Recording

The simulation is recorded using PyVista’s movie writer:

```python
plotter.open_movie("missile_vs_target.mp4", framerate=50)
```

* Output format: **MP4**
* Frame rate: **50 FPS**
* Saved automatically in the project directory

---

## 🧩 Dependencies

Install required packages:

```bash
pip install numpy pandas trimesh pyvista vtk tkinter
```

> ⚠️ **Note**: On Linux, Tkinter may need to be installed separately:

```bash
sudo apt install python3-tk
```

---

## ▶️ How to Run

```bash
python3 pyvista.py
```

1. A file dialog will appear
2. Select your trajectory CSV file
3. The simulation window opens
4. Animation plays and records automatically
5. MP4 video is saved on completion

---

## 🎮 Controls

* Rotate view: **Left mouse button**
* Pan: **Middle mouse button**
* Zoom: **Scroll wheel**

---
## 🎥 Output Video
https://github.com/user-attachments/assets/fefc4a72-dd58-4f6d-8f23-8631319689d1

## ⚙️ Customization

| Parameter      | Location      | Description         |
| -------------- | ------------- | ------------------- |
| `KILL_DIST`    | Hit detection | Lethal radius       |
| `scale`        | GLTF loader   | Model size          |
| `MISSILE_ROT`  | Axis fix      | Missile orientation |
| `AIRCRAFT_ROT` | Axis fix      | Target orientation  |
| `framerate`    | Movie writer  | Video FPS           |

---

## 🛑 Common Issues

### Blank Model

✔ Ensure GLTF models are **centered and scaled** correctly

### No Video Output

✔ `plotter.write_frame()` must be inside animation loop

### Tkinter Error

✔ Install `python3-tk`

---

## 📌 Tested On

* Python 3.10+
* Linux (X11)
* PyVista 0.43+
* VTK 9+

---

## 📜 License

This project is intended for **simulation, visualization, and educational use only**.

---

## ✨ Author

**Poloju Shiva Charan Chary**
Missile Trajectory & 3D Simulation

---

