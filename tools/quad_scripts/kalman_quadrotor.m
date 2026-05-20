% Discrete Kalman Filter for Quadrotor UAV

clc; clear; close all;

%%Discrete system matrices
A = [1,      0.05,   0,      0,      0,      0,      0,       0,      0.0123, 0.0002, 0,      0;
     0,      1,      0,      0,      0,      0,     -0.0123,  0,      0.4905, 0.0123, 0,      0;
     0,      0,      1,      0.05,   0,      0,      0,      -0.0123, 0,      0,      0,      0;
     0,      0,      0,      1,      0,      0,     -0.4905, -0.0123, 0,      0,      0,      0;
     0,      0,      0,      0,      1,      0.05,   0,       0,      0,      0,      0,      0;
     0,      0,      0,      0,      0,      1,      0,       0,      0,      0,      0,      0;
     0,      0,      0,      0,      0,      0,      1,       0.05,   0,      0,      0,      0;
     0,      0,      0,      0,      0,      0,      0,       1,      0,      0,      0,      0;
     0,      0,      0,      0,      0,      0,      0,       0,      1,      0.05,   0,      0;
     0,      0,      0,      0,      0,      0,      0,       0,      0,      1,      0,      0;
     0,      0,      0,      0,      0,      0,      0,       0,      0,      0,      1,      0.05;
     0,      0,      0,      0,      0,      0,      0,       0,      0,      0,      0,      1];

B = [0,       0,       0,       0;
    -0.0002,  0,       0.0002,  0;
     0,       0,       0,       0;
     0,       0.0002,  0,      -0.0002;
     0.0001,  0.0001,  0.0001,  0.0001;
     0.0021,  0.0021,  0.0021,  0.0021;
     0,      -0.0012,  0,       0.0012;
     0,      -0.0472,  0,       0.0472;
    -0.0012,  0,       0.0012,  0;
    -0.0472,  0,       0.0472,  0;
     0.0001, -0.0001,  0.0001, -0.0001;
     0,       0,       0,       0];

C = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0];

% Noise covariances (slide 39)
% Process noise: v_k = 0.09 * ones(12,1)
Q = 0.09 * eye(12);

% Measurement noise: w_k = [0.25,0,0.25,0,0.25,0,0.3,0,0.3,0,0.3,0]'
% -> covariance for the 6 outputs [x, y, z, phi, theta, psi]
R = diag([0.25, 0.25, 0.25, 0.3, 0.3, 0.3]);

% System initial conditions
T  = 0.01;
t  = 0:T:30;
N  = length(t);
nu = 4;  % number of inputs
nx = 12; % number of states
ny = 6;  % number of outputs

u = sin(t) .* ones(nu, N);  % u = sin(nT) applied to all 4 inputs

x0 = zeros(nx, 1);
x  = zeros(nx, N);  x(:,1)  = x0;
y  = zeros(ny, N);  y(:,1)  = C * x0;

% Kalman filter initial conditions
xh = zeros(nx, 1);  % initial a posteriori estimate
xp = zeros(nx, N);  % a priori estimates
xh_hist = zeros(nx, N);
xh_hist(:,1) = xh;
xp(:,1)      = xh;
P = eye(nx);        % initial covariance P0

%%Simulation — non-stationary Kalman filter
for n = 1:N-1
    % --- True system (with noise) ---
    x(:,n+1) = A*x(:,n) + B*u(:,n) + sqrt(0.09)*randn(nx,1);
    y(:,n+1) = C*x(:,n+1) + chol(R)'*randn(ny,1);

    %  Kalman filter 
    % Prediction (a priori estimate)
    xp(:,n+1) = A*xh_hist(:,n) + B*u(:,n);

    % A priori covariance
    S = A*P*A' + Q;

    % Kalman gain
    L = S*C' * inv(C*S*C' + R);

    % Correction (a posteriori estimate)
    xh_hist(:,n+1) = xp(:,n+1) + L*(y(:,n+1) - C*xp(:,n+1));

    % Covariance update (Joseph form — numerically stable)
    ILC = eye(nx) - L*C;
    P = ILC*S*ILC' + L*R*L';
end

%%  Plots 
% State indices: x=1, xdot=2, y=3, ydot=4, z=5, zdot=6,
%                phi=7, phidot=8, theta=9, thetadot=10, psi=11, psidot=12

pose_idx = [1, 3, 5, 7, 9, 11];
vel_idx  = [2, 4, 6, 8, 10, 12];
pose_labels = {'$x$ (m)', '$y$ (m)', '$z$ (m)', '$\phi$ (rad)', '$\theta$ (rad)', '$\psi$ (rad)'};
vel_labels  = {'$\dot{x}$ (m/s)', '$\dot{y}$ (m/s)', '$\dot{z}$ (m/s)', ...
               '$\dot{\phi}$ (rad/s)', '$\dot{\theta}$ (rad/s)', '$\dot{\psi}$ (rad/s)'};

%  Figure 1: Translational pose (x, y, z) 
figure('Name','Pose - Translational Position');
state_names = {'$x$', '$y$', '$z$'};
for i = 1:3
    idx = pose_idx(i);
    subplot(3,1,i);
    plot(t, x(idx,:), 'Color',[0.5 0.5 0.5], 'LineWidth',1, 'DisplayName','True'); hold on;
    plot(t, xh_hist(idx,:), 'b', 'LineWidth',1.5, 'DisplayName','KF Estimate');
    plot(t, xp(idx,:), 'r--', 'LineWidth',1, 'DisplayName','A priori prediction');
    ylabel(pose_labels{i}, 'Interpreter','latex'); grid on;
    legend('Location','best');
    title(['Translational position - ' state_names{i}], 'Interpreter','latex');
end
xlabel('Time (s)');

%  Figure 2: Rotational pose (phi, theta, psi) 
figure('Name','Pose - Angles (Roll, Pitch, Yaw)');
angle_names = {'Roll ($\phi$)', 'Pitch ($\theta$)', 'Yaw ($\psi$)'};
for i = 1:3
    idx = pose_idx(i+3);
    subplot(3,1,i);
    plot(t, x(idx,:), 'Color',[0.5 0.5 0.5], 'LineWidth',1, 'DisplayName','True'); hold on;
    plot(t, xh_hist(idx,:), 'b', 'LineWidth',1.5, 'DisplayName','KF Estimate');
    plot(t, xp(idx,:), 'r--', 'LineWidth',1, 'DisplayName','A priori prediction');
    ylabel(pose_labels{i+3}, 'Interpreter','latex'); grid on;
    legend('Location','best');
    title(['Angle - ' angle_names{i}], 'Interpreter','latex');
end
xlabel('Time (s)');

%  Figure 3: Translational velocities (xdot, ydot, zdot) 
figure('Name','Velocities - Translational');
vt_names = {'$\dot{x}$', '$\dot{y}$', '$\dot{z}$'};
for i = 1:3
    idx = vel_idx(i);
    subplot(3,1,i);
    plot(t, x(idx,:), 'Color',[0.5 0.5 0.5], 'LineWidth',1, 'DisplayName','True'); hold on;
    plot(t, xh_hist(idx,:), 'b', 'LineWidth',1.5, 'DisplayName','KF Estimate');
    ylabel(vel_labels{i}, 'Interpreter','latex'); grid on;
    legend('Location','best');
    title(['Translational velocity - ' vt_names{i}], 'Interpreter','latex');
end
xlabel('Time (s)');

%  Figure 4: Angular velocities (phidot, thetadot, psidot) 
figure('Name','Velocities - Angular');
va_names = {'$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$'};
for i = 1:3
    idx = vel_idx(i+3);
    subplot(3,1,i);
    plot(t, x(idx,:), 'Color',[0.5 0.5 0.5], 'LineWidth',1, 'DisplayName','True'); hold on;
    plot(t, xh_hist(idx,:), 'b', 'LineWidth',1.5, 'DisplayName','KF Estimate');
    ylabel(vel_labels{i+3}, 'Interpreter','latex'); grid on;
    legend('Location','best');
    title(['Angular velocity - ' va_names{i}], 'Interpreter','latex');
end
xlabel('Time (s)');

%  Figure 5: Noisy measurement vs KF estimate (sensor outputs) 
figure('Name','Noisy Measurements vs KF Estimate (C*x outputs)');
out_labels = {'$x$', '$y$', '$z$', '$\phi$', '$\theta$', '$\psi$'};
for i = 1:6
    subplot(3,2,i);
    plot(t, y(i,:), 'k.', 'MarkerSize',2, 'DisplayName','Noisy measurement'); hold on;
    plot(t, xh_hist(pose_idx(i),:), 'b', 'LineWidth',1.5, 'DisplayName','KF Estimate');
    ylabel(out_labels{i}, 'Interpreter','latex'); grid on;
    legend('Location','best');
    title(out_labels{i}, 'Interpreter','latex');
end
xlabel('Time (s)');
