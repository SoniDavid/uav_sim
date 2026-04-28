# Quadrotor UAV Simulator — IRS 6to Tarea 3

Mathematical + ROS2 simulation of a 45 g quadrotor.
Replaces MATLAB/Simulink with Python + ROS2 Humble + Foxglove Studio.

---

## Table of Contents

1. [Project structure](#1-project-structure)
2. [Physical model and parameters](#2-physical-model-and-parameters)
3. [Dynamics implementation](#3-dynamics-implementation)
4. [Mixer matrix](#4-mixer-matrix)
5. [Controller design](#5-controller-design)
6. [ROS2 node and topics](#6-ros2-node-and-topics)
7. [Quick start — standalone plots (no ROS2)](#7-quick-start--standalone-plots-no-ros2)
8. [Quick start — ROS2 + Foxglove](#8-quick-start--ros2--foxglove)
9. [Foxglove layout setup](#9-foxglove-layout-setup)
10. [Launch arguments](#10-launch-arguments)
11. [Phase 2 — Gazebo Harmonic](#11-phase-2--gazebo-harmonic)
- Needed Changes: [TO DO](#TO-DO)


---

## 1. Project structure

```
UAV_sim/
│
├── src/quadrotor_sim/              ← ROS2 Python package
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/quadrotor_sim
│   ├── launch/
│   │   └── sim.launch.py           ← starts simulator + foxglove_bridge
│   ├── config/
│   │   └── foxglove_layout.json    ← pre-configured Foxglove panels
│   └── quadrotor_sim/
│       ├── params.py               ← all physical constants (single source of truth)
│       ├── dynamics.py             ← 16-state nonlinear ODE + RK4 integrator
│       ├── mixer.py                ← (T, τ_φ, τ_θ, τ_ψ) ↔ motor speeds
│       ├── controller.py           ← LQR design + cascaded nonlinear PD
│       ├── simulator_node.py       ← ROS2 node (main entry point)
│       └── test_sim.py             ← standalone simulation + matplotlib plots
│
└── gazebo/                         ← Phase 2 (Gazebo Harmonic)
    ├── models/quadrotor/
    │   ├── model.config
    │   └── model.sdf               ← SDF with MulticopterMotorModel plugins
    └── worlds/
        └── quadrotor.world
```

The ROS2 workspace is at `~/ros2_ws/src/quadrotor_sim` (symlink to `src/quadrotor_sim`).

---

## 2. Physical model and parameters

All constants live in `params.py`.

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Total mass | m | 0.045 | kg |
| Gravity | g | 9.81 | m/s² |
| CG to motor shaft | L | 0.061 | m |
| Diagonal span | S | 0.15 | m |
| Effective moment arm | ARM = S/2 | 0.075 | m |
| Roll inertia | I_xx | 7.921 × 10⁻⁵ | kg·m² |
| Pitch inertia | I_yy | 13.604 × 10⁻⁵ | kg·m² |
| Yaw inertia | I_zz | 7.317 × 10⁻⁵ | kg·m² |
| Rotor inertia | J_r | 1.4961 × 10⁻⁷ | kg·m² |
| Thrust coefficient | k_T | 2.64 × 10⁻⁸ | N/(rad/s)² |
| Torque coefficient | k_Q | 5.4 × 10⁻⁹ | Nm/(rad/s)² |
| Motor time constant | T_m | 0.07 | s |
| Max rotor speed | ω_max | 2640 | rad/s |
| Hover thrust | T_hover = mg | 0.441 | N |
| Max thrust (all motors) | T_max = 4·k_T·ω_max² | 0.736 | N |
| Hover rotor speed | ω_hover = √(mg/4k_T) | ~2044 | rad/s |

**Thrust headroom:** T_max - T_hover = 0.295 N (66% above hover).
This is tight and is the primary constraint on controller aggressiveness.

---

## 3. Dynamics implementation

`dynamics.py` — 16-state ODE integrated with a fixed-step RK4 at 1 kHz.

### State vector

```
state[0:3]   = [x, y, z]               position in inertial frame (m)
state[3:6]   = [ẋ, ẏ, ż]              linear velocity (m/s)
state[6:9]   = [φ, θ, ψ]              Euler angles ZYX convention (rad)
state[9:12]  = [p, q, r]               body angular rates (rad/s)
state[12:16] = [ω₁, ω₂, ω₃, ω₄]      rotor speeds (rad/s)
```

### Motor dynamics

First-order filter (from `params.py` model `t_m·ω̇ = k_m·u - ω`):

```
ω̇ᵢ = (ω_des,i − ωᵢ) / T_m
```

The control command `u` is the **desired rotor speed in rad/s** (not a normalized [0,1] signal).
This avoids the ambiguity in the PDF's `k_m = 684` parameter — using desired omega directly is equivalent and unambiguous.

### Aerodynamic forces

```
Tᵢ = k_T · ωᵢ²       individual motor thrust
Qᵢ = k_Q · ωᵢ²       individual motor reaction torque
T  = T₁ + T₂ + T₃ + T₄   total thrust
```

### Motor layout (X-configuration, viewed from above)

```
    Motor 1 (CW)    Motor 2 (CCW)
     (-x, +y)         (+x, +y)
          \             /
           \           /
            [base_link]
           /           \
          /             \
     (-x, -y)         (+x, -y)
    Motor 4 (CCW)    Motor 3 (CW)
```

### Body moments

```
τ_φ = ARM · (T₂ + T₃ − T₁ − T₄)      roll
τ_θ = ARM · (T₁ + T₂ − T₃ − T₄)      pitch
τ_ψ = −Q₁ + Q₂ − Q₃ + Q₄             yaw
```

### Translational dynamics (inertial frame)

```
ẍ = (T/m) · (cos φ · sin θ · cos ψ + sin φ · sin ψ)
ÿ = (T/m) · (cos φ · sin θ · sin ψ − sin φ · cos ψ)
z̈ = (T/m) · cos φ · cos θ − g
```

### Rotational dynamics (with gyroscopic term)

```
Ω_gyro = ω₁ − ω₂ + ω₃ − ω₄   (net rotor speed, alternating signs)

I_xx · ṗ = (I_yy − I_zz)·q·r + J_r·q·Ω_gyro + τ_φ
I_yy · q̇ = (I_zz − I_xx)·p·r − J_r·p·Ω_gyro + τ_θ
I_zz · ṙ = (I_xx − I_yy)·p·q               + τ_ψ
```

### Kinematics (body rates → Euler angle rates)

ZYX Euler convention. Warns if |θ| > 88° (near gimbal lock):

```
[φ̇]   [1  sin(φ)tan(θ)  cos(φ)tan(θ)] [p]
[θ̇] = [0  cos(φ)       −sin(φ)      ] [q]
[ψ̇]   [0  sin(φ)/cos(θ) cos(φ)/cos(θ)] [r]
```

---

## 4. Mixer matrix

`mixer.py` — converts between high-level commands and individual motor speeds.

### Forward mixer (A_mix)

Maps `[ω₁², ω₂², ω₃², ω₄²]` → `[T, τ_φ, τ_θ, τ_ψ]`:

```
        ω₁²        ω₂²        ω₃²        ω₄²
T    [  k_T        k_T        k_T        k_T   ]
τ_φ  [ -ARM·k_T   ARM·k_T    ARM·k_T   -ARM·k_T]
τ_θ  [  ARM·k_T   ARM·k_T   -ARM·k_T   -ARM·k_T]
τ_ψ  [ -k_Q       k_Q       -k_Q        k_Q    ]
```

### Inverse mixer

`A_mix_inv` (precomputed via `np.linalg.inv`) maps the desired `[T, τ_φ, τ_θ, τ_ψ]` back to `ωᵢ²`, then `ωᵢ = √(ωᵢ²)` clipped to `[0, ω_max]`.

---

## 5. Controller design

`controller.py` contains **two controllers**. Only the cascaded one is used at runtime; the LQR is used for the linearized analysis required in the report.

### 5a. LQR (for report / state-space analysis)

Linearized at hover: φ = θ = ψ = 0, T = mg, all rates = 0.

At hover the system decouples into four double-integrator chains:

**A matrix (12×12) — nonzero entries:**

```
A[0,3]=1  A[1,4]=1  A[2,5]=1     (position ← velocity)
A[6,9]=1  A[7,10]=1  A[8,11]=1   (angle ← angular rate)
A[3,7]=g                          (ẍ ← g·θ,  pitch drives x)
A[4,6]=-g                         (ÿ ← −g·φ, roll drives y)
```

**B matrix (12×4) — nonzero entries:**

```
B[5,0]  = 1/m        (z̈   ← δT)
B[9,1]  = 1/I_xx     (ṗ   ← τ_φ)
B[10,2] = 1/I_yy     (q̇   ← τ_θ)
B[11,3] = 1/I_zz     (ṙ   ← τ_ψ)
```

**CARE solution** (`scipy.linalg.solve_continuous_are`):

```
P = CARE(A, B, Q, R)
K = R⁻¹ · Bᵀ · P        shape (4, 12)
```

**Q and R (Bryson's rule):**

```
Q = diag([4, 4, 4,  4, 4, 4,  14, 14, 4,  3.7, 3.7, 3.7])
         pos(m)    vel(m/s)   ang(rad)     rate(rad/s)

R = diag([11.5,  330,  330,  46])
          δT(N)  τ_φ   τ_θ   τ_ψ   (Nm)
```

Verified: all closed-loop eigenvalues of (A − B·K) have Re < 0.

**LQR limitation:** Due to the drone's tight thrust budget (only 66% headroom), the LQR saturates motors during large reference steps. It is suitable for the linearized analysis but requires a trajectory pre-filter and tilt compensation for the full nonlinear simulation.

### 5b. Cascaded nonlinear PD (used at runtime)

Standard quadrotor cascaded controller with **tilt compensation**. Handles the nonlinear coupling between attitude and altitude that breaks the pure LQR on this vehicle.

**Outer position loop (PD):**

```python
ax_d = Kp_xy·(x_r − x) + Kd_xy·(ẋ_r − ẋ)     # desired x acceleration
ay_d = Kp_xy·(y_r − y) + Kd_xy·(ẏ_r − ẏ)
az_d = Kp_z ·(z_r − z) + Kd_z ·(ż_r − ż)

Kp_xy = 2.0,  Kd_xy = 1.5
Kp_z  = 4.0,  Kd_z  = 3.0
```

**Tilt-compensated total thrust:**

```python
# When tilted, only cos(φ)·cos(θ) fraction of thrust acts vertically.
# Increase T to compensate so altitude is maintained during lateral motion.
T = m · (az_d + g) / (cos(φ)·cos(θ))
T = clip(T, 0, T_max)
```

**Desired attitude from desired acceleration:**

```python
θ_d = arcsin( m·ax_d / T )           # pitch to go in x
φ_d = arcsin( −m·ay_d / (T·cos θ_d)) # roll to go in y  
ψ_d = ψ_ref
```

**Inner attitude loop (PD):**

```python
τ_φ = I_xx · (Kp_att·(φ_d − φ) − Kd_att·p)
τ_θ = I_yy · (Kp_att·(θ_d − θ) − Kd_att·q)
τ_ψ = I_zz · (Kp_psi·(ψ_d − ψ) − Kd_psi·r)

Kp_att = 300,  Kd_att = 30
Kp_psi = 100,  Kd_psi = 15
```

**Reference trajectory smoother:**

To avoid actuator saturation during waypoint steps, the reference position is rate-limited at 0.2 m/s before being sent to the controller:

```python
x_ref[i] += clip(target[i] − x_ref[i], −v_max·dt, v_max·dt)
```

**Test scenario result:**
- t = 0–5 s: climb to hover at [0, 0, 1] m
- t = 5–~15 s: move to [1, 1, 2] m
- Angles stay within ±1° throughout; z never drops more than a few cm.

---

## 6. ROS2 node and topics

`simulator_node.py` — timer at 50 Hz, runs 20 internal RK4 steps (1 kHz) per callback.

### Topics published at 50 Hz

| Topic | Type | Content |
|---|---|---|
| `/uav/pose` | `geometry_msgs/PoseStamped` | Position (m) + orientation (quaternion) |
| `/uav/euler` | `geometry_msgs/Vector3Stamped` | φ, θ, ψ in radians |
| `/uav/state` | `std_msgs/Float64MultiArray` | Full 12-state vector (see order above) |
| `/uav/motors` | `std_msgs/Float64MultiArray` | [T₁, T₂, T₃, T₄] in Newtons |
| `/uav/motor_omega` | `std_msgs/Float64MultiArray` | [ω₁, ω₂, ω₃, ω₄] in rad/s |
| `/uav/reference` | `geometry_msgs/PoseStamped` | Current smoothed waypoint |

### TF tree

```
map (static) → world → base_link
```

`map → world` is a static identity transform published once at startup to give Foxglove a stable root frame and prevent 3D panel flickering.
`world → base_link` is broadcast at 50 Hz from the simulator state.

---

## 7. Quick start — standalone plots (no ROS2)

Generates the 6-panel matplotlib figure required for the assignment report.

```bash
cd UAV_sim/src/quadrotor_sim
python3 -m quadrotor_sim.test_sim
```

Output saved to `/tmp/quadrotor_lqr_results.png`.

The figure shows (20 s simulation, reference switch at t = 5 s):
- Position: x, y, z vs time
- Euler angles: φ, θ, ψ vs time
- Linear velocities: ẋ, ẏ, ż vs time
- Angular rates: ṗ, q̇, ṙ vs time
- Motor thrusts: T₁, T₂, T₃, T₄ vs time
- Motor speed commands: ω_des₁..₄ vs time

---

## 8. Quick startT

### Always kill old nodes before relaunching

ROS2 does **not** stop previous instances when you re-run a launch file. Two simulator nodes publishing to the same topics will cause flickering and zigzag plots.

```bash
pkill -f simulator; pkill -f foxglove_bridge; sleep 1
```

Or add this alias to `~/.bashrc`:

```bash
alias uav_launch='pkill -f simulator; pkill -f foxglove_bridge; sleep 1; ros2 launch quadrotor_sim sim.launch.py'
```

### Launch — simulator only (manual position commands)

```bash
pkill -f simulator; pkill -f foxglove_bridge; sleep 1
ros2 launch quadrotor_sim sim.launch.py
```

Then send position targets at any time from another terminal:

```bash
ros2 topic pub --once /uav/cmd_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 1.5}}}'
```

### Launch — simulator + YAML trajectory (automatic)

```bash
pkill -f simulator; pkill -f foxglove_bridge; sleep 1
ros2 launch quadrotor_sim sim.launch.py use_trajectory:=true
```

Uses `config/trajectory.yaml` by default. To supply a custom file:

```bash
ros2 launch quadrotor_sim sim.launch.py \
  use_trajectory:=true \
  trajectory_file:=/absolute/path/to/my_trajectory.yaml
```

To loop the trajectory forever:

```bash
ros2 launch quadrotor_sim sim.launch.py use_trajectory:=true loop:=true
```

### Trajectory YAML format

```yaml
trajectory:
  - position: [x, y, z]   # metres
    hold_time: 5.0         # seconds to hold before next waypoint
  - position: [1.0, 0.0, 1.0]
    hold_time: 4.0
  # ... add as many waypoints as needed
```

The reference smoother (`ref_max_vel`) handles smooth transitions between waypoints.

```bash
cd ~/Documents/classes/IRS_6to/UAV_sim
colcon build --packages-select quadrotor_sim --symlink-install
```

---

## 9. Foxglove layout setup

### Import the pre-configured layout

In Foxglove Studio:
1. Top-right → click the layout name → **Import from file**
2. Select: `UAV_sim/src/quadrotor_sim/config/foxglove_layout.json`

### Panel contents after import

| Panel | What it shows |
|---|---|
| 3D (top-left) | Drone model moving in world frame via TF |
| Plot — Position | x, y, z and their references vs time |
| Plot — Euler Angles | φ (roll), θ (pitch), ψ (yaw) in radians |
| Plot — Motor Thrusts | T₁, T₂, T₃, T₄ in Newtons |
| Plot — Motor Speeds | ω₁, ω₂, ω₃, ω₄ in rad/s |

### Subscribing to individual state fields

`/uav/state` is a `Float64MultiArray`. In Foxglove Plot panels, index fields like:

```
/uav/state.data[0]  → x       /uav/state.data[6]  → φ (roll)
/uav/state.data[1]  → y       /uav/state.data[7]  → θ (pitch)
/uav/state.data[2]  → z       /uav/state.data[8]  → ψ (yaw)
/uav/state.data[3]  → ẋ       /uav/state.data[9]  → p (roll rate)
/uav/state.data[4]  → ẏ       /uav/state.data[10] → q (pitch rate)
/uav/state.data[5]  → ż       /uav/state.data[11] → r (yaw rate)
```

---

## 9b. RViz — 3D drone model (custom URDF)

Export your CAD model as a URDF (e.g. from SolidWorks with the SW2URDF plugin, or Fusion 360 with the Fusion2URDF plugin). The URDF's root link must be named `base_link` — the simulator already publishes the `world → base_link` TF. 

Once you have your URDF, pass it to the launch file:

```bash
ros2 launch quadrotor_sim sim.launch.py \
  urdf_file:=/absolute/path/to/your_drone.urdf
```

This starts `robot_state_publisher` automatically and publishes `/robot_description`.

Then open RViz:

```bash
rviz2
```

Inside RViz:
1. **Fixed Frame** → `world`
2. **Add → RobotModel** → set *Description Topic* to `/robot_description`
3. (Optional) **Add → TF** to see coordinate frames

**URDF requirements for compatibility:**
- Root link named `base_link`
- All mesh paths either absolute or using `package://` URIs
- If using meshes (`.stl`, `.dae`), place them in `urdf/meshes/` and install via `setup.py`

---

## 10. Launch arguments

| Argument | Default | Description |
|---|---|---|
| `ref_x` | `0.0` | Initial target x (m) |
| `ref_y` | `0.0` | Initial target y (m) |
| `ref_z` | `1.0` | Initial target z (m) |
| `ref_max_vel` | `0.5` | Reference ramp speed (m/s) |
| `controller` | `cascaded` | Controller mode |
| `use_trajectory` | `false` | Enable trajectory runner |
| `trajectory_file` | (default YAML) | Path to trajectory YAML |
| `loop` | `false` | Loop trajectory on completion |

**Controller options:** `cascaded` \| `lqr` \| `constrained_lqr` \| `constrained_lqr_qp` \| `gain_scheduled_lqr`

Examples:

```bash
# LQR controller with trajectory
ros2 launch quadrotor_sim sim.launch.py controller:=lqr use_trajectory:=true

# Slower reference tracking
ros2 launch quadrotor_sim sim.launch.py ref_max_vel:=0.2

# Start at different altitude
ros2 launch quadrotor_sim sim.launch.py ref_z:=2.0
```

---

## 11. Phase 2 — Gazebo Harmonic

Files are ready but require Gazebo Harmonic + `ros_gz_bridge` to be installed.

### Model

`gazebo/models/quadrotor/model.sdf` — four rotor links with `MulticopterMotorModel` plugins using the exact aerodynamic parameters (k_T, k_Q, T_m, ω_max).

Motor layout in SDF (rotor positions computed from ARM = S/2 = 0.075 m at 45°):

```
Rotor 1 (CW):  pose −0.053, +0.053   (front-left)
Rotor 2 (CCW): pose +0.053, +0.053   (front-right)
Rotor 3 (CW):  pose +0.053, −0.053   (rear-right)
Rotor 4 (CCW): pose −0.053, −0.053   (rear-left)
```

### Launch with Gazebo

```bash
# Set model path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/gazebo/models

# Start Gazebo
gz sim gazebo/worlds/quadrotor.world

# In a second terminal, bridge topics to ROS2
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /motor_speed@std_msgs/msg/Float64MultiArray@gz.msgs.Double_V \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

The Python controller (`simulator_node.py`) reads the ROS2 odometry topic and commands motor speeds through the bridge — no changes to controller logic needed.

---

## Known issues and design decisions

### Why cascaded PD instead of pure LQR at runtime?

This drone has only **66% thrust headroom** above hover (T_max = 0.736 N, T_hover = 0.441 N). During a 1 m position step, the LQR commands up to 14 N of thrust — 48× beyond the physical limit. Motor saturation makes the closed-loop behavior completely different from the linear model.

The cascaded PD with tilt compensation handles this correctly because:
1. It explicitly accounts for the nonlinear tilt-altitude coupling (`T = m·(az + g) / cos(φ)·cos(θ)`)
2. The reference trajectory smoother limits position error to a size the actuators can handle
3. Each sub-loop (position, attitude) is tuned independently and intuitively

The LQR linearized model and gain computation are preserved in `controller.py` and used in `test_sim.py` pre-flight checks for the state-space diagram in the report.

### Motor command units

The reference identifies the motor model as `t_m·ω̇ + ω = k_m·u` with k_m = 684.
At hover: `u = ω_hover / k_m = 2044 / 684 ≈ 3.0` — not in [0, 1].

Rather than carry the ambiguous normalization, the implementation uses `ω_des` (rad/s) directly as the motor command:

```
ω̇ = (ω_des − ω) / T_m
```

This is mathematically equivalent with k_m = 1 and u = ω_des.

---

## TO DO
- Change structure implementation and 2D
- COnfigure and valid gazebo implementation
- Add URDF to simulation
- Implement this with hexacopter


