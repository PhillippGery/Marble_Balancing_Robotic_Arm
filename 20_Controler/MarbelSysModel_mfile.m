syms x x_dot y y_dot alpha alpha_dot beta beta_dot alpha_ddot beta_ddot real


%% 1. Parameters
g = 9.81; 
mb = 0.05; % kg
rb = 0.015; % m
Ib = (2/5)*mb*rb^2;
C = 5/7*g; % Constant from (mb*g) / (mb + Ib/rb^2)

%% 2. Nonlinear Model (for ode45)
 x = [x, x_dot, y, y_dot, alpha, alpha_dot, beta, beta_dot]
 u = [alpha_ddot, beta_ddot]
f_nonlin = [
    x(2);                                       % dx/dt = ball_velocity_x
    (mb*x(1)*x(6)^2 + mb*x(3)*x(6)*x(8) - mb*g*sin(x(5))) / (mb + Ib/rb^2); % d_vx/dt
    x(4);                                       % dy/dt = ball_velocity_y
    (mb*x(3)*x(8)^2 + mb*x(1)*x(6)*x(8) - mb*g*sin(x(7))) / (mb + Ib/rb^2); % d_vy/dt
    x(6);                                       % d_alpha/dt = plate_angular_velocity_x
    u(1);                                       % d_alphadot/dt = input_alpha_ddot
    x(8);                                       % d_beta/dt = plate_angular_velocity_y
    u(2)                                        % d_betadot/dt = input_beta_ddot
];
simplify(f_nonlin)

f_nonlin_handle = @(t, x, u) f_nonlin;

%% 3. Compute Jacobians (Taylor Linearization)
% Using transpose (') to ensure x and u are column vectors for the Jacobian calculation
A_sym = jacobian(f_nonlin, x');
B_sym = jacobian(f_nonlin, u');

%% 4. Evaluate at Equilibrium (x=0, u=0)
% Define numeric equilibrium point
x_eq = zeros(8,1);
u_eq = zeros(2,1);

% Substitute equilibrium and parameters
A_numeric = double(subs(A_sym, [x'; u'], [x_eq; u_eq]));
B_numeric = double(subs(B_sym, [x'; u'], [x_eq; u_eq]));

disp(A_numeric);
disp(B_numeric);

%% 4. LQR Controller Design
Q = diag([1000, 100, 1000, 100, 1, 0.1, 1, 0.1]); 
R = eye(2) * 0.01;
K = lqr(A_numeric, B_numeric, Q, R)
% %% 5. Simulation on Nonlinear Model
% f_numeric = matlabFunction(f_nonlin, 'Vars', {x.', u.'}); 
% 
% % STEP B: Define the closed-loop dynamics: dx/dt = f(x, -K*x)
% % This satisfies the "Module 2" requirement for integrating planning and control
% cl_dynamics = @(t, x_val) f_numeric(x_val, -K * x_val);
% 
% % STEP C: Run the numeric simulation using ode45
% x0 = [0.1; 0; -0.1; 0; 0; 0; 0; 0]; % Initial 10cm offset in X and Y
% tspan = 0:0.01:5;
% [t, states] = ode45(cl_dynamics, tspan, x0);
% 
% %% 6. Visualization (Essential for Milestone 3)
% figure('Name', 'Nonlinear Labyrinth Balancing');
% subplot(2,1,1);
% plot(t, states(:,1), 'r', t, states(:,3), 'b', 'LineWidth', 1.5);
% title('Ball Position: Real-Time Stabilization');
% xlabel('Time (s)'); ylabel('Position (m)'); 
% legend('x-pos', 'y-pos'); grid on;
% 
% subplot(2,1,2);
% plot(t, rad2deg(states(:,5)), 'r--', t, rad2deg(states(:,7)), 'b--', 'LineWidth', 1.5);
% title('Plate Angles: Control Effort (Degrees)');
% xlabel('Time (s)'); ylabel('Angle (deg)'); 
% legend('\alpha (tilt x)', '\beta (tilt y)'); grid on;

% %% 7. Top-Down Trajectory Plot (Ball Path)
% figure('Name', 'Top-Down View of Ball Trajectory');
% hold on;
% 
% % Plot the path taken by the ball
% plot(states(:,1), states(:,3), 'm', 'LineWidth', 2); 
% 
% % Mark the Start and End points
% plot(states(1,1), states(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
% plot(0, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2);                             % Goal
% 
% % Plot the boundaries of the plate (assuming a 0.5m x 0.5m plate)
% rectangle('Position', [-0.25, -0.25, 0.5, 0.5], 'EdgeColor', 'k', 'LineStyle', '--');
% 
% % Formatting the plot
% xlabel('X Position (m)');
% ylabel('Y Position (m)');
% title('Top-Down View: Ball Path to Equilibrium');
% legend('Ball Trajectory', 'Initial Position', 'Target (0,0)');
% axis equal; % Ensures the spatial aspect ratio is correct
% grid on;
% hold off;

%% 5. Simulation with Lissajous Trajectory Tracking
% Parameters for Lissajous Curve (Adjust for speed/shape)
A_amp = 0.2; B_amp = 0.2; % 10cm amplitude
a_freq = 0.6; b_freq = a_freq*2; 
delta = 0;             % Phase shift

% Step A: Convert symbolic f_nonlin to numeric function
f_numeric = matlabFunction(f_nonlin, 'Vars', {x.', u.'}); 

% Step B: Define Tracking Dynamics
cl_dynamics = @(t, x_val) tracker_dynamics(t, x_val, f_numeric, K, A_amp, B_amp, a_freq, b_freq, delta);

% Step C: Run Simulation
x0 = [A_amp*sin(delta); 0; 0; 0; 0; 0; 0; 0]; % Start at the first point of the curve
[n, m] = rat(a_freq/b_freq); 
T_total = (2 * pi * m) / a_freq;
tspan = 0:0.01:T_total; % Longer simulation to see the full curve
[t, states] = ode45(cl_dynamics, tspan, x0);

%% Helper Function for Tracking 
function dxdt = tracker_dynamics(t, x_curr, f_num, K, A, B, a, b, d)
    % 1. Define reference positions
    xr = A * sin(a*t + d);
    yr = B * sin(b*t);
    
    % 2. Define reference velocities (derivatives of positions)
    vxr = A * a * cos(a*t + d);
    vyr = B * b * cos(b*t);
    
    % 3. Build the reference state vector [x; vx; y; vy; a; da; b; db]
    % We set target angles and angular velocities to 0 
    x_ref = [xr; vxr; yr; vyr; 0; 0; 0; 0];
    
    % 4. Compute LQR Control Law: u = -K * (error)
    u_lqr = -K * (x_curr - x_ref);
    
    % 5. Evaluate nonlinear dynamics
    dxdt = f_num(x_curr, u_lqr);
end

%% 8. Visualization of Tracking Performance
figure('Name', 'Lissajous Tracking Performance');
% Top-Down Plot
subplot(1,2,1);
plot(states(:,1), states(:,3), 'm', 'LineWidth', 2); hold on;
% Calculate true reference for plotting
t_ref = linspace(0, 10, 500);
plot(A_amp*sin(a_freq*t_ref + delta), B_amp*sin(b_freq*t_ref), 'k--', 'LineWidth', 1);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Top-Down View: Lissajous Curve');
legend('Ball Path', 'Desired Path'); axis equal; grid on;

% Time-Series Plot
subplot(1,2,2);
plot(t, states(:,1), 'r', t, A_amp*sin(a_freq*t + delta), 'k--'); hold on;
plot(t, states(:,3), 'b', t, B_amp*sin(b_freq*t), 'k--');
xlabel('Time (s)'); ylabel('Position (m)');
title('X and Y Tracking over Time');
legend('X Actual', 'X Ref', 'Y Actual', 'Y Ref'); grid on;


%% 10. Plate State Analysis: Angles and Angular Velocities
% Convert plate states to Degrees and Degrees per Second
alpha_deg = rad2deg(states(:,5));       % x-tilt angle
beta_deg  = rad2deg(states(:,7));       % y-tilt angle
alpha_dot_deg = rad2deg(states(:,6));   % x-tilt angular velocity
beta_dot_deg  = rad2deg(states(:,8));   % y-tilt angular velocity

figure('Name', 'Plate Dynamics: Orientation and Velocity');

% Subplot 1: Plate Angles (Position)
subplot(2,1,1);
plot(t, alpha_deg, 'r', 'LineWidth', 1.5); hold on;
plot(t, beta_deg, 'b', 'LineWidth', 1.5);
ylabel('Angle (deg)');
title('Plate Orientation (State: \alpha, \beta)');
legend('Alpha (x-axis)', 'Beta (y-axis)');
grid on;

% Subplot 2: Plate Angular Velocities (Speed)
subplot(2,1,2);
plot(t, alpha_dot_deg, 'r--', 'LineWidth', 1.5); hold on;
plot(t, beta_dot_deg, 'b--', 'LineWidth', 1.5);
ylabel('Angular Velocity (deg/s)');
xlabel('Time (s)');
title('Plate Angular Velocity (State: \dot{\alpha}, \dot{\beta})');
legend('d\alpha/dt', 'd\beta/dt');
grid on;