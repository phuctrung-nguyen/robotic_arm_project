# 4-DOF RPRR Welding Robot — Robotics Course Project

Design, analysis and simulation of a 4-degree-of-freedom welding robot arm of type **R–P–R–R** (revolute – prismatic – revolute – revolute), covering the full pipeline from mechanical design, kinematics, dynamics, trajectory planning and controller design to simulation in ROS 2.

## Contents

| Folder | Content | Tools |
|---|---|---|
| [`solid_des`](solid_des) | 3D models of the links, assemblies, welding head, technical drawings | SolidWorks |
| [`mapler`](maple) | Derivation of the robot's differential equations of motion | Maple |
| [`trajectory`](trajectory) | Forward/inverse kinematics (DH), Jacobian, workspace, manipulability index, LSPB trajectories in joint space and Cartesian space, animation | Python |
| [`IDPD_simulink`](IDPD_simulink) | IDPD (Inverse Dynamics PD) trajectory-tracking controller | MATLAB / Simulink |
| [`sim_ros2`](sim_ros2) | URDF model, Gazebo + RViz simulation, per-joint position control using Gazebo's PID | ROS 2 Jazzy, Gazebo Harmonic |

## Robot parameters (DH)

| Joint | Type | d (m) | a (m) | α (°) | Limits |
|---|---|---|---|---|---|
| q1 | Revolute | 0.2175 | 0 | 0 | 0° → 360° |
| q2 | Prismatic | q2 | 0.25 | 90 | 0 → 0.225 m |
| q3 | Revolute (θ = q3 + 90°) | 0.105 | 0 | 90 | −90° → 135° |
| q4 | Revolute (welding torch) | 0.262 | 0 | 0 | 0° → 360° |

Fixed base d0 = 0.084 m. The welding torch tip is offset by (−68, 0, 126) mm from frame O4.

End-effector convention: the workspace (`main.py`) is computed for the **welding torch tip** (with the offset above); the welding trajectory, inverse kinematics, Simulink and the ROS 2 simulation are all computed for the **origin of frame O4** (q4 = 0).

**Forward kinematics** (origin of frame O4):

```
x = 0.25·cos q1 + 0.105·sin q1 + 0.262·cos q1·cos q3
y = 0.25·sin q1 − 0.105·cos q1 + 0.262·sin q1·cos q3
z = q2 + 0.262·sin q3 + 0.3015
```

**Inverse kinematics**:

```
q1 = atan2(0.105, √(x² + y² − 0.105²)) + atan2(y, x)
q3 = atan2(√(1 − D²), D),   D = (x·cos q1 + y·sin q1 − 0.25) / 0.262
q2 = z − 0.262·sin q3 − 0.3015
```

This model is consistent across the whole repo: the DH table (`config/dh_params.yaml`), the URDF in ROS 2, the Python trajectory scripts and the Gazebo control node.

## Welding cycle

Non-contact arc welding along a 0.2 m vertical segment BA. The Cartesian trajectory uses a trapezoidal velocity profile (LSPB) with a sampling period of 0.01 s.

| Point | Coordinates (m) | Joint variables (q1 rad, q2 m, q3 rad) |
|---|---|---|
| HOME | (0.1518, −0.2730, 0.7698) | (−0.7205, 0.2101, 1.4013) |
| B — weld start | (0.1589, −0.1543, 0.7782) | (−0.2768, 0.2205, 1.7822) |
| A — weld end | (0.1589, −0.1543, 0.5782) | (−0.2768, 0.0205, 1.7822) |

| Phase | Max velocity | Acceleration | End time |
|---|---|---|---|
| HOME → B | 0.05 m/s | 0.10 m/s² | 2.88 s |
| Dwell at B (arc ignition) | — | — | 3.38 s |
| B → A (welding) | 0.01 m/s | 0.05 m/s² | 23.58 s |
| Dwell at A (crater filling) | — | — | 24.58 s |
| A → HOME | 0.05 m/s | 0.10 m/s² | 29.60 s |

2961 reference points in total (`Cartesian_Waypoints.txt`). The corresponding joint trajectories are exported to Simulink (`Vitridat.mat`, `Vantocdat.mat`, `Giatocdat.mat`).

## Quick start

**Kinematics & trajectories (Python)**

```bash
cd 03_dong_hoc_quy_dao_python
pip install -r requirements.txt
python main.py            # workspace + manipulability -> data/ket_qua.png
python JointSpace.py      # joint trajectories + export Vitridat/Vantocdat/Giatocdat.mat for Simulink
python CartesianSpace.py  # Cartesian trajectory + export Cartesian_Waypoints.txt
python animation.py       # 3D animation of the robot following the trajectory
python -m src.run_simulation  # quick Jacobian check at a sample configuration
```

**IDPD controller (MATLAB)**: run `initialize.m` first to load the parameters, then open `controller_simulation.slx` and click Run. See the [README](04_bo_dieu_khien_IDPD_simulink/README.md).

**ROS 2 simulation**: see [05_mo_phong_ros2/README.md](05_mo_phong_ros2/README.md).

## License

[MIT](LICENSE)

## Results

![Workspace and manipulability](03_dong_hoc_quy_dao_python/data/ket_qua.png)
