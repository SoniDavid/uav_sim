import numpy as np

# Physical parameters
MASS = 0.045        # kg
G    = 9.81         # m/s²
L    = 0.061        # m, CG to motor shaft
S    = 0.15         # m, diagonal span between rotor pairs
ARM  = S / 2.0      # 0.075 m, effective moment arm in mixer

# Inertia tensor (diagonal)
IXX = 7.921e-5      # kg·m²
IYY = 13.604e-5     # kg·m²
IZZ = 7.317e-5      # kg·m²
JR  = 1.4961e-7     # kg·m², rotor inertia

# Aerodynamic coefficients (experimental identification)
KT = 2.64e-8        # N/(rad/s)²  thrust coefficient
KQ = 5.4e-9         # Nm/(rad/s)² torque coefficient

# Motor first-order model
TM        = 0.01    # s, motor time constant (reduced from 0.07 for stability at 50 Hz control)
KM        = 684.0   # rad/s per normalized input unit
OMEGA_MAX = 2640.0  # rad/s, maximum rotor speed
OMEGA_MIN = 0.0     # rad/s

# Simulation timing
DT_SIM            = 0.01                # s, RK4 integration step (100 Hz)
DT_PUB            = 0.02                # s, ROS2 publish period (50 Hz)
SIM_STEPS_PER_PUB = int(DT_PUB / DT_SIM)  # 2 steps per publish

# Hover equilibrium values
T_HOVER     = MASS * G                        # 0.44145 N total thrust
OMEGA_HOVER = np.sqrt(T_HOVER / (4.0 * KT))  # ~2044 rad/s per motor
