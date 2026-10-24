



# Clearpath Ridgeback Autonomous 3D Navigation Simulation

Welcome to the **Clearpath Ridgeback 3D Navigation Simulation** in MATLAB & Simscape Multibody.

This package contains everything needed to simulate the 3D physics-based navigation of the Clearpath Ridgeback Omnidirectional / Mecanum AMR in a large warehouse arena.

---

## 🌟 What's New in This Release

1. **Legacy "Solid Brick" Removed**:
   - The default legacy `Brick Solid` block ($15\text{ m} \times 15\text{ m}$) from the original URDF import has been completely removed from both `ridgeback.slx` and `ridgeback_3d_nav.slx`.
   - The world now exclusively features the clean $25\text{ m} \times 25\text{ m}$ warehouse arena floor (`Env_Ground_Floor`) with no overlapping visual bricks.

2. **Calibrated Wheel-to-Ground Contact ($Z = 0.00\text{ m}$)**:
   - The wheels no longer sink or penetrate into the floor surface.
   - Configured with a dedicated `RT_BaseLink_Height` transform ($[0, 0, +0.22\text{ m}]$) to place the wheel treads exactly tangent to the floor surface.

3. **Interactive Click-to-Navigate GUI**:
   - Launching the simulation presents an interactive 2D map.
   - **Click anywhere on the map** with your mouse to instantly pick a new target destination!
   - The system automatically recalculates the collision-free path (Hybrid A*), computes smooth trajectory dynamics (Pure Pursuit), updates the 3D target beacon, and runs Mechanics Explorer in 3D.

4. **All Files Organized in a Single Dedicated Folder**:
   - All models, scripts, URDF definitions, and tools are self-contained in this single folder.

---

## 🚀 Quick Start: How to Run

### Option 1: Run in MATLAB (Recommended)

1. Open **MATLAB R2025b**.
2. Set your current directory to this folder by executing in the Command Window:
   ```matlab
   cd('C:\Users\HARINI\Downloads\Ridgeback_3D_Simulation')
   ```
3. Launch the interactive simulation:
   ```matlab
   run_ridgeback_3d_nav
   ```
4. **Interactive Controls**:
   - **Click anywhere on the map**: Sets a new target point, plans a collision-free path, and automatically triggers the 3D simulation.
   - **Quick Preset Buttons**: Click any preset button on the right panel (e.g., `📍 Far Corner [22, 22]`, `📍 Central Plaza [12.5, 12.5]`, `📍 South-East Bay [18, 9]`, `📍 North-West Bay [8, 18]`).
   - **▶ Replay 3D Simulation**: Replays the existing trajectory in Mechanics Explorer.

---

### Option 2: Run via Python

If you prefer controlling the simulation from Python:
```bash
cd C:\Users\HARINI\Downloads\Ridgeback_3D_Simulation
python run_3d_nav.py --goal 20.0 20.0
```

---

## 📁 File Manifest

| File | Description |
| :--- | :--- |
| **[`run_ridgeback_3d_nav.m`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/run_ridgeback_3d_nav.m)** | **Main interactive launcher**. Renders the 2D arena map with live click-to-navigate event callbacks and 3D Mechanics Explorer visualization. |
| **[`build_ridgeback_3d_simulation.m`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/build_ridgeback_3d_simulation.m)** | Core automated model builder. Assembles the $25\text{ m} \times 25\text{ m}$ arena, 10 obstacles, start & target beacons, planar joint, height calibration, Hybrid A* planner, and Pure Pursuit controller. |
| **[`ridgeback_3d_nav.slx`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/ridgeback_3d_nav.slx)** | Verified 3D Simscape Multibody model containing the entire arena, physics joint actuation, obstacles, and robot. |
| **[`ridgeback.slx`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/ridgeback.slx)** | Cleaned base robot Simscape model (with legacy brick solid completely removed). |
| **[`ridgeback.urdf`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/ridgeback.urdf)** | Clearpath Ridgeback AMR URDF description file. |
| **[`simulink_connector.py`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/simulink_connector.py)** | Python connector to automate MATLAB/Simulink sessions. |
| **[`run_3d_nav.py`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/run_3d_nav.py)** | Python command-line launcher for 3D navigation. |
| **[`navigate_ridgeback_nav2.m`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/navigate_ridgeback_nav2.m)** | Standalone 2D navigation algorithm reference. |
| **[`ridgeback_navigation_simulation.m`](file:///C:/Users/HARINI/Downloads/Ridgeback_3D_Simulation/ridgeback_navigation_simulation.m)** | Comprehensive 2D path planning and trajectory evaluation script. |

---

## 🗺️ Arena Layout & Coordinates

- **Arena Size**: $25.0\text{ m} \times 25.0\text{ m}$ enclosed by 4 boundary walls.
- **Start Position**: Green cylindrical pad & beacon at $[3.0\text{ m}, 3.0\text{ m}]$.
- **Target Position**: Dynamic red cylindrical pad & beacon flag at user-selected coordinates (default $[22.0\text{ m}, 22.0\text{ m}]$).
- **Obstacle Set**:
  - 4 Warehouse Shelving Racks ($4.0\text{ m} \times 1.5\text{ m}$)
  - 2 Central Plaza Crate Blocks ($2.5\text{ m} \times 2.5\text{ m}$)
  - 2 Cylindrical Concrete Columns (Radius $0.5\text{ m}$)
  - 2 Pallet Stacks ($2.0\text{ m} \times 2.0\text{ m}$)
- **Chassis Elevation**: $+0.22\text{ m}$ so wheels are flush on floor with zero penetration.
