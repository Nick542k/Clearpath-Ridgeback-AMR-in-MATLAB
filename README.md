# Clearpath Ridgeback Autonomous 3D Navigation Simulation

<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-R2025b%20%7C%20R2024b-0076A8?style=for-the-badge&logo=mathworks&logoColor=white" alt="MATLAB" />
  <img src="https://img.shields.io/badge/Simulink-Simscape%20Multibody-E65100?style=for-the-badge&logo=mathworks&logoColor=white" alt="Simulink" />
  <img src="https://img.shields.io/badge/Platform-Clearpath%20Ridgeback%20AMR-orange?style=for-the-badge" alt="Platform" />
  <img src="https://img.shields.io/badge/Navigation-Hybrid%20A*%20%2B%20Pure%20Pursuit-4CAF50?style=for-the-badge" alt="Navigation" />
  <img src="https://img.shields.io/badge/License-MIT-blue?style=for-the-badge" alt="License" />
</p>

---

## 🎬 3D Simulation Demonstration

https://github.com/user-attachments/assets/d88c35a9-6eda-4245-b001-cec74772b3ba

> *Real-time physics-based 3D Mechanics Explorer simulation showing the Clearpath Ridgeback AMR navigating an expanded $25\text{ m} \times 25\text{ m}$ warehouse arena with full-map routing and zero-penetration wheel contact.*

---

## 📖 Overview

This repository provides an autonomous 3D digital-twin navigation simulation for the **Clearpath Ridgeback Autonomous Mobile Robot (AMR)** built with **MATLAB**, **Simulink**, and **Simscape Multibody**.

The simulation combines kinodynamic **Hybrid A\*** global path planning, **Pure Pursuit** motion control, and multi-DOF Simscape Multibody dynamics inside an expanded $25\text{ m} \times 25\text{ m}$ warehouse environment. It features an interactive **Click-to-Navigate GUI** that allows users to click anywhere on the arena map to dynamically compute collision-free routes and observe the 3D mechanics in Simscape Mechanics Explorer in real time.

---

## 🌟 Key Features

- **🎯 Interactive Click-to-Navigate GUI**:
  - Click anywhere inside the $25\text{ m} \times 25\text{ m}$ arena map with your mouse to instantly set a new destination.
  - Automatically replans the collision-free path, updates the 3D target beacon, and triggers the 3D physics simulation.
  - Includes quick-target preset buttons (`Far Corner`, `Central Plaza`, `South-East Bay`, `North-West Bay`) and a simulation replay control.

- **🛞 Calibrated Wheel-to-Ground Contact ($Z = 0.00\text{ m}$)**:
  - Precise chassis elevation via `RT_BaseLink_Height` ($[0, 0, +0.22\text{ m}]$) compensates for the wheel axle offset ($-0.10\text{ m}$) and wheel radius ($0.12\text{ m}$).
  - Eliminates visual floor penetration: wheel treads sit perfectly tangent on the floor surface.

- **🗺️ Expanded $25\text{ m} \times 25\text{ m}$ Warehouse Environment**:
  - 4 perimeter boundary walls ($1.2\text{ m}$ high).
  - 10 distinct obstacle bodies: 4 heavy-duty shelving racks, 2 central crate stacks, 2 cylindrical concrete columns, and 2 pallet stacks.
  - Generous interior aisles ensuring whole-map utilization without artificial wall-hugging.

- **📍 Dynamic 3D Visual Beacons**:
  - **Start Marker**: Green circular pad + vertical beacon mast + flag located at $[3.0\text{ m}, 3.0\text{ m}]$.
  - **Goal Marker**: Red circular pad + vertical beacon mast + flag dynamically positioned at the active target destination.

- **🧠 Kinodynamic Navigation Pipeline**:
  - **Global Planner**: Hybrid A\* kinodynamic search on an inflated occupancy grid (turning radius $1.2\text{ m}$, step $1.0\text{ m}$).
  - **Trajectory Tracking**: Pure Pursuit controller computing synchronized linear velocity and yaw rate.
  - **Planar Actuation**: 3-DOF `MobileBaseJoint` ($P_x, P_y, R_z$) actuated by 2nd-order filtered Simscape-PS signals with automatically computed reaction torques.

- **🛡️ Zero-Error Auto-Load Callbacks**:
  - `InitFcn` and `PreLoadFcn` model callbacks automatically load `default_trajectory.mat` into the base workspace if variables are missing.
  - The model can be run from the GUI or directly by pressing the green **Run (▶)** button in Simulink without missing-variable errors.

---

## 🏗️ System Architecture

```
                      ┌────────────────────────────────────────┐
                      │        Interactive MATLAB GUI          │
                      │       (run_ridgeback_3d_nav.m)         │
                      └──────────────────┬─────────────────────┘
                                         │ Click Event (X, Y)
                                         ▼
                      ┌────────────────────────────────────────┐
                      │    Hybrid A* Kinodynamic Path Planner  │
                      │    (build_ridgeback_3d_simulation.m)   │
                      └──────────────────┬─────────────────────┘
                                         │ Collision-Free Waypoints
                                         ▼
                      ┌────────────────────────────────────────┐
                      │       Pure Pursuit Controller          │
                      │  (Generates sim_traj_x, y, yaw vs t)   │
                      └──────────────────┬─────────────────────┘
                                         │ Trajectory Signals
                                         ▼
                      ┌────────────────────────────────────────┐
                      │    Simscape Multibody 3D Physics       │
                      │       (ridgeback_3d_nav.slx)           │
                      │  - MobileBaseJoint (InputMotion)       │
                      │  - RT_BaseLink_Height (+0.22m contact) │
                      │  - 25m x 25m Arena & 10 Obstacles      │
                      └──────────────────┬─────────────────────┘
                                         │ 3D Visualization
                                         ▼
                      ┌────────────────────────────────────────┐
                      │      Simscape Mechanics Explorer       │
                      │     (Real-Time 3D Animation)          │
                      └────────────────────────────────────────┘
```

---

## 🚀 Getting Started

### Prerequisites

- **MATLAB R2024b** or **MATLAB R2025b**
- **Simulink**
- **Simscape** & **Simscape Multibody**
- **Navigation Toolbox**
- **ROS Toolbox** / **Robotics System Toolbox** (for URDF parsing)

---

### How to Run

#### Option 1: Interactive Click-to-Navigate GUI (Recommended)

1. Open MATLAB.
2. In the MATLAB Command Window, navigate to the project directory:
   ```matlab
   cd('C:\Users\HARINI\Downloads\Ridgeback_3D_Simulation')
   ```
3. Run the interactive launcher:
   ```matlab
   run_ridgeback_3d_nav
   ```
4. **Controls**:
   - **Click anywhere on the map** to pick a new destination.
   - Use the **preset buttons** on the right side to jump to popular warehouse zones.
   - Watch **Mechanics Explorer** animate the robot in 3D!

---

#### Option 2: Direct Simulink Run

1. Open [`ridgeback_3d_nav.slx`](ridgeback_3d_nav.slx) directly in Simulink.
2. Click the green **Run (▶)** button.
3. The built-in `InitFcn` callback will automatically load the default trajectory and run the simulation with zero errors.

---

#### Option 3: Python Automation API

To run the simulation headlessly or batch test different target coordinates via Python:

```bash
cd C:\Users\HARINI\Downloads\Ridgeback_3D_Simulation
python run_3d_nav.py --goal 20.0 20.0
```

---

## 🗺️ Arena Layout & Coordinates

| Element | Position $(X, Y, Z)$ [m] | Dimensions / Radius [m] | Color |
| :--- | :--- | :--- | :--- |
| **Ground Floor** | $[12.5, 12.5, -0.05]$ | $25.0 \times 25.0 \times 0.1$ | Slate Gray |
| **Boundary Walls** | South, North, West, East | Height: $1.2\text{ m}$, Thickness: $0.25\text{ m}$ | Dark Charcoal |
| **Start Marker Pad** | $[3.0, 3.0, 0.005]$ | Radius: $0.9\text{ m}$, Beacon: $1.2\text{ m}$ | Vivid Green |
| **Target Marker Pad** | Dynamic $[X_g, Y_g, 0.005]$ | Radius: $0.9\text{ m}$, Beacon: $1.2\text{ m}$ | Vivid Red + Flag |
| **Shelving Racks (x4)** | SW, SE, NW, NE | $4.0 \times 1.5 \times 1.8$ | Warehouse Blue |
| **Central Crates (x2)** | $[10.25, 14.25]$, $[14.75, 10.75]$ | $2.5 \times 2.5 \times 1.5$ | Cargo Orange |
| **Pillars (x2)** | West $[6.5, 12.5]$, East $[18.5, 12.5]$ | Radius: $0.5\text{ m}$, Height: $2.5\text{ m}$ | Industrial Teal |
| **Pallet Stacks (x2)** | $[4.5, 10.0]$, $[20.5, 15.0]$ | $2.0 \times 2.0 \times 1.2$ | Timber Brown |

---

## 📁 Repository Structure

```
Ridgeback_3D_Simulation/
├── README.md                          # Comprehensive documentation & video demo
├── run_ridgeback_3d_nav.m             # Main 1-click interactive map GUI
├── build_ridgeback_3d_simulation.m    # Automated model builder & trajectory planner
├── ridgeback_3d_nav.slx               # Complete 3D Simscape Multibody model
├── ridgeback.slx                      # Clean base robot Simscape model
├── ridgeback.urdf                     # URDF robot description
├── default_trajectory.mat             # Persistent fallback trajectory
├── run_3d_nav.py                      # Python command-line runner
├── simulink_connector.py              # Python-MATLAB engine interface bridge
├── navigate_ridgeback_nav2.m          # Standalone 2D navigation algorithm
├── ridgeback_navigation_simulation.m  # 2D benchmark and path evaluation script
├── .gitignore                         # MATLAB / Simulink / Python ignore rules
└── .gitattributes                     # Binary file tracking rules
```

---

## ⚙️ Technical Details

### Wheel-to-Ground Calibration Math
The Ridgeback robot base link frame is located above the wheel axles:
$$\text{Axle Offset } Z_{\text{axle}} = -0.10\text{ m}$$
$$\text{Wheel Radius } R_{\text{wheel}} = 0.12\text{ m}$$
$$\text{Lowest Tread Point } Z_{\text{bottom}} = Z_{\text{axle}} - R_{\text{wheel}} = -0.10 - 0.12 = -0.22\text{ m}$$

By inserting a `Rigid Transform` block (`RT_BaseLink_Height`) with translation $[0, 0, +0.22\text{ m}]$ between the `MobileBaseJoint` follower frame and `base_link`, the lowest wheel tread is positioned precisely at $Z = 0.00\text{ m}$, flush with the top surface of the arena floor.

### Actuation and Dynamics
The planar motion is driven by a `Planar Joint` configured with:
- **Motion Actuation**: `InputMotion` on $P_x, P_y, R_z$.
- **Torque Mode**: `ComputedTorque` automatically resolving dynamic forces.
- **Signal Filtering**: Second-order input filtering with time constant $\tau = 0.02\text{ s}$ via `Simulink-PS Converter` blocks for smooth acceleration and jitter-free multibody solver execution.

---

## 📜 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing & Acknowledgments

- **Clearpath Robotics** for the original Ridgeback mobile robot URDF and specifications.
- **MathWorks** for Simscape Multibody, Navigation Toolbox, and Mechanics Explorer.
