% Quadrotor UAV — State-Space Linearization at Hover
%
% State:  x = [x  y  z  xdot  ydot  zdot  phi  theta  psi  phidot  thetadot  psidot]' (12x1)
% Input:  u = [delta_T  tau_phi  tau_theta  tau_psi]'  (4x1)
%           delta_T   : thrust deviation from hover (N)
%           tau_phi   : roll torque  (N·m)
%           tau_theta : pitch torque (N·m)
%           tau_psi   : yaw torque   (N·m)
% Output: y = C*x + D*u  (full state, C = I, D = 0)
%
% Linearization point: hover (phi=theta=psi=0, T = m*g)
% Valid for small angles: |phi|, |theta| < ~20 deg

% Parameters
m   = 0.045;          % mass (kg)
g   = 9.81;           % gravity (m/s²)
Ixx = 7.921e-5;       % roll  inertia (kg·m²)
Iyy = 13.604e-5;      % pitch inertia (kg·m²)
Izz = 7.317e-5;       % yaw   inertia (kg·m²)
kT  = 2.64e-8;        % thrust coefficient  N/(rad/s)²
kQ  = 5.40e-9;        % torque coefficient  N·m/(rad/s)²
ARM = 0.075;          % CG-to-motor arm length (m)  [S/2 = 0.15/2]
w_max = 2640;         % max rotor speed (rad/s)

T_hover = m * g;                    % hover thrust (N)
w_hover = sqrt(T_hover / (4*kT));   % hover rotor speed (rad/s)
T_max   = 4 * kT * w_max^2;        % max total thrust (N)

fprintf('=== Quadrotor Parameters ===\n')
fprintf('  m      = %.4f kg\n', m)
fprintf('  g      = %.4f m/s²\n', g)
fprintf('  Ixx    = %.3e kg·m²\n', Ixx)
fprintf('  Iyy    = %.3e kg·m²\n', Iyy)
fprintf('  Izz    = %.3e kg·m²\n', Izz)
fprintf('  ARM    = %.4f m\n', ARM)
fprintf('  T_hover= %.4f N\n', T_hover)
fprintf('  w_hover= %.2f rad/s\n', w_hover)
fprintf('  T_max  = %.4f N\n\n', T_max)

% A matrix (12×12)
% Row/column order: x y z xd yd zd phi theta psi phid thetad psid
%                   1 2 3  4  5  6   7     8   9   10     11   12

A = zeros(12, 12);

% Kinematics: velocity integrates into position
A(1, 4)  = 1;     % dx/dt    = xdot
A(2, 5)  = 1;     % dy/dt    = ydot
A(3, 6)  = 1;     % dz/dt    = zdot

% Euler angle kinematics (small-angle: J_inv ≈ I)
A(7, 10) = 1;     % dphi/dt   = phidot
A(8, 11) = 1;     % dtheta/dt = thetadot
A(9, 12) = 1;     % dpsi/dt   = psidot

% Gravity-attitude coupling (linearized around hover)
A(4, 8)  =  g;    % d(xdot)/dt ≈  g * theta
A(5, 7)  = -g;    % d(ydot)/dt ≈ -g * phi

% B matrix (12×4) 
% Input order: delta_T  tau_phi  tau_theta  tau_psi
%                  1        2        3          4

B = zeros(12, 4);

B(6,  1) = 1/m;    % d(zdot)/dt    = delta_T / m
B(10, 2) = 1/Ixx;  % d(phidot)/dt  = tau_phi   / Ixx
B(11, 3) = 1/Iyy;  % d(thetadot)/dt= tau_theta / Iyy
B(12, 4) = 1/Izz;  % d(psidot)/dt  = tau_psi   / Izz

%  C and D matrices 
% Full-state output (perfect knowledge in simulation, no sensors modeled)
C = eye(12);        % y = x  (all states directly observable)
D = zeros(12, 4);   % no direct feedthrough (inputs affect accelerations, not states)

% Display 
state_names = {'x','y','z','xdot','ydot','zdot','phi','theta','psi','phidot','thetadot','psidot'};
input_names = {'dT','t_phi','t_th','t_psi'};

col_w  = 10;   % column width for A and K (values <= 9.81)
col_wb = 14;   % column width for B (values up to ~13667)
lbl_w  = 10;   % row label width

fprintf('A matrix (12×12) — system dynamics: xdot = A*x + B*u\n')
fprintf('%*s', lbl_w, '');
for j = 1:12; fprintf('%*s', col_w, state_names{j}); end
fprintf('\n')
for i = 1:12
    fprintf('%-*s', lbl_w, state_names{i})
    for j = 1:12
        if A(i,j) == 0
            fprintf('%*s', col_w, '.');
        else
            fprintf('%*.4f', col_w, A(i,j));
        end
    end
    fprintf('\n')
end

fprintf('\nB matrix (12×4) — input coupling\n')
fprintf('%*s', lbl_w, '');
for j = 1:4; fprintf('%*s', col_wb, input_names{j}); end
fprintf('\n')
for i = 1:12
    fprintf('%-*s', lbl_w, state_names{i})
    for j = 1:4
        if B(i,j) == 0
            fprintf('%*s', col_wb, '.');
        else
            fprintf('%*.4f', col_wb, B(i,j));
        end
    end
    fprintf('\n')
end

fprintf('\nC matrix (12×12) — output selection: y = C*x\n')
fprintf('  C = eye(12)  [full state output — all states directly observable]\n')

fprintf('\nD matrix (12×4) — feedthrough: y += D*u\n')
fprintf('  D = zeros(12,4)  [no direct feedthrough — inputs affect accelerations only]\n')

% Open-loop eigenvalues
eigs_ol = eig(A);
fprintf('\n Open-loop eigenvalues of A \n')
for i = 1:length(eigs_ol)
    fprintf('  lambda_%02d = %+.4f + %.4fj\n', i, real(eigs_ol(i)), imag(eigs_ol(i)))
end
fprintf('  -> All at origin (marginally stable, as expected for a free-floating body)\n')

% LQR design 
% Bryson rule weights (same as Python controller.py)
Q = diag([4, 4, 4, 4, 4, 4, 14, 14, 4, 3.7, 3.7, 3.7]);
R = diag([11.5, 330.0, 330.0, 46.0]);

K = lqr(A, B, Q, R);

fprintf('\n K matrix (4×12) — LQR gain: u = -K*(x - x_ref) \n')
fprintf('%*s', lbl_w, '');
for j = 1:12; fprintf('%*s', col_w, state_names{j}); end
fprintf('\n')
for i = 1:4
    fprintf('%-*s', lbl_w, input_names{i})
    for j = 1:12
        if abs(K(i,j)) < 1e-10
            fprintf('%*s', col_w, '.');
        else
            fprintf('%*.4f', col_w, K(i,j));
        end
    end
    fprintf('\n')
end

A_cl = A - B*K;
eigs_cl = eig(A_cl);
fprintf('\n Closed-loop eigenvalues of (A - B*K) \n')
all_stable = true;
for i = 1:length(eigs_cl)
    if real(eigs_cl(i)) >= 0
        all_stable = false;
        fprintf('  lambda_%02d = %+.4f + %.4fj  UNSTABLE\n', i, real(eigs_cl(i)), imag(eigs_cl(i)))
    else
        fprintf('  lambda_%02d = %+.4f + %.4fj\n', i, real(eigs_cl(i)), imag(eigs_cl(i)))
    end
end
if all_stable
    fprintf('  -> Closed-loop system is STABLE (all Re(lambda) < 0)\n')
end

% Build ss object 
sys_ol = ss(A, B, C, D);
sys_cl = ss(A_cl, B*0, C, D);   % autonomous closed-loop (no external input)

fprintf('\n State-space objects created \n')
fprintf('  sys_ol : open-loop  plant  (A, B, C, D)\n')
fprintf('  sys_cl : closed-loop plant (A-BK, 0, C, D)\n')
fprintf('  K      : LQR gain matrix (4x12)\n')
