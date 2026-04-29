%% Inverted Pendulum on a Cart - H2/LQG Project Version
% This version upgrades the original homework-style script into a project:
% 1) reference tracking
% 2) nonlinear plant simulation
% 3) observer-based control
% 4) value logging
% 5) animation

clear; clc; close all;

%% 1) Parameters
p.M = 2;          % cart mass [kg]
p.m = 1;          % pendulum mass [kg]
p.l = 4;          % pendulum length [m]
p.g = 9.8;        % gravity [m/s^2]

dt   = 0.005;     % simulation step [s]
tEnd = 20;        % total time [s]
t    = (0:dt:tEnd).';
N    = numel(t);

uMax = 20;        % actuator force limit [N]

%% 2) Linearized model about the upright equilibrium
% state x = [theta; theta_dot; x_c; x_c_dot]

A = [ 0                              1   0   0;
      ((p.M+p.m)*p.g)/(p.M*p.l)      0   0   0;
      0                              0   0   1;
     -(p.m*p.g)/p.M                  0   0   0 ];

B = [0;
    -1/(p.M*p.l);
     0;
     1/p.M];

% Disturbance channel
Bd = B;

% Measured outputs
% More practical project choice: measure theta and cart position
C = [1 0 0 0;
     0 0 1 0];

% Tracking output: cart position
Ctrack = [0 0 1 0];

%% 3) Controller design: servo LQR (H2/LQG style)
% Add one integral state xi = integral(r - x_c) to remove steady-state error

Aaug = [A,            zeros(4,1);
       -Ctrack,       0          ];
Baug = [B;
        0];

% Tune these weights
Qservo = diag([80 10 200 20 400]);
Rservo = 0.8;

Kaug = lqr(Aaug, Baug, Qservo, Rservo);

Kx = Kaug(1:4);   % state feedback
Ki = Kaug(5);     % integral action

disp('State feedback gain Kx = ');
disp(Kx);
disp('Integral gain Ki = ');
disp(Ki);

%% 4) Observer design: Kalman filter / LQE
% H2/LQG flavor consistent with your original code

G = eye(4);  % process noise distribution

Qn = diag([1e-4 1e-2 1e-4 1e-2]);     % process noise covariance
Rn = diag([5e-4 5e-4]);               % measurement noise covariance

L = lqe(A, G, C, Qn, Rn);

disp('Observer gain L = ');
disp(L);

%% 5) Reference signal and disturbance
r = zeros(N,1);

% Example reference motion:
% 0 m -> 0.5 m -> -0.4 m -> 0 m
for k = 1:N
    if t(k) >= 2 && t(k) < 8
        r(k) = 0.5;
    elseif t(k) >= 8 && t(k) < 14
        r(k) = -0.4;
    elseif t(k) >= 14
        r(k) = 0;
    end
end

% External disturbance force
d = zeros(N,1);
d(t >= 6 & t <= 6.2) = 1.5;   % short disturbance pulse

% Measurement noise
rng(1);   % reproducible
noise = [0.002*randn(N,1), 0.003*randn(N,1)];

%% 6) Initialization
x    = zeros(4,N);    % true states
xhat = zeros(4,N);    % estimated states
y    = zeros(2,N);    % measured outputs
u    = zeros(N,1);    % control input
e    = zeros(N,1);    % tracking error
xi   = zeros(N,1);    % integral state

% Initial condition: pendulum starts with 5 deg deviation
x(:,1)    = [5*pi/180; 0; 0; 0];
xhat(:,1) = [0; 0; 0; 0];
xi(1)     = 0;

%% 7) Simulation loop
for k = 1:N-1

    % Measured output
    y(:,k) = C*x(:,k) + noise(k,:)';

    % Tracking error based on measured cart position
    e(k) = r(k) - y(2,k);

    % Observer-based control law
    u_unsat = -Kx*xhat(:,k) - Ki*xi(k);

    % Saturation
    u(k) = min(max(u_unsat, -uMax), uMax);

    % Simple anti-windup
    if abs(u_unsat - u(k)) < 1e-9
        xi(k+1) = xi(k) + dt*e(k);
    else
        xi(k+1) = xi(k);
    end

    % Nonlinear plant update with disturbance
    uPlant = u(k) + d(k);
    x(:,k+1) = rk4_step(@(xs,uu) cartpole_nonlinear(xs,uu,p), x(:,k), uPlant, dt);

    % Observer update (linear observer)
    xhat_dot = A*xhat(:,k) + B*u(k) + L*(y(:,k) - C*xhat(:,k));
    xhat(:,k+1) = xhat(:,k) + dt*xhat_dot;
end

% Last sample
y(:,N) = C*x(:,N) + noise(N,:)';
e(N)   = r(N) - y(2,N);
u(N)   = u(N-1);

%% 8) Performance summary
thetaDeg = x(1,:)*180/pi;
maxThetaDeg = max(abs(thetaDeg));
maxU = max(abs(u));
rmseTrack = sqrt(mean((r - x(3,:)').^2));

fprintf('\n================ Performance Summary ================\n');
fprintf('Max |theta|          = %.4f deg\n', maxThetaDeg);
fprintf('Max |u|              = %.4f N\n',   maxU);
fprintf('Tracking RMSE (x_c)  = %.6f m\n',   rmseTrack);
fprintf('Final cart position  = %.6f m\n',   x(3,end));
fprintf('Final pendulum angle = %.6f deg\n', x(1,end)*180/pi);

%% 9) Save log data
logTable = table( ...
    t, ...
    x(1,:)', x(2,:)', x(3,:)', x(4,:)', ...
    xhat(1,:)', xhat(2,:)', xhat(3,:)', xhat(4,:)', ...
    y(1,:)', y(2,:)', ...
    r, e, u, d, ...
    'VariableNames', { ...
    't', ...
    'theta', 'theta_dot', 'xc', 'xc_dot', ...
    'theta_hat', 'theta_dot_hat', 'xc_hat', 'xc_dot_hat', ...
    'theta_meas', 'xc_meas', ...
    'xc_ref', 'tracking_error', 'u', 'disturbance'});

writetable(logTable, 'cart_pendulum_log.csv');
disp('Log data saved to cart_pendulum_log.csv');

%% 10) Plots
figure('Name','Inverted Pendulum Project','Color','w');
tiledlayout(3,2);

nexttile;
plot(t, x(1,:)*180/pi, 'b', 'LineWidth', 1.4); hold on; grid on;
plot(t, xhat(1,:)*180/pi, 'b--', 'LineWidth', 1.2);
ylabel('\theta (deg)');
legend('\theta', '\theta hat', 'Location', 'best');
title('Pendulum Angle');

nexttile;
plot(t, x(2,:), 'b', 'LineWidth', 1.4); hold on; grid on;
plot(t, xhat(2,:), 'b--', 'LineWidth', 1.2);
ylabel('\theta dot (rad/s)');
legend('\theta dot', '\theta dot hat', 'Location', 'best');
title('Angular Velocity');

nexttile;
plot(t, x(3,:), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, xhat(3,:), 'r--', 'LineWidth', 1.2);
plot(t, r, 'k:', 'LineWidth', 1.5);
ylabel('x_c (m)');
legend('x_c', 'x_c hat', 'x_c ref', 'Location', 'best');
title('Cart Position Tracking');

nexttile;
plot(t, x(4,:), 'r', 'LineWidth', 1.4); hold on; grid on;
plot(t, xhat(4,:), 'r--', 'LineWidth', 1.2);
ylabel('x_c dot (m/s)');
legend('x_c dot', 'x_c dot hat', 'Location', 'best');
title('Cart Velocity');

nexttile;
plot(t, u, 'g', 'LineWidth', 1.4); hold on; grid on;
yline(uMax, 'k--');
yline(-uMax, 'k--');
xlabel('Time (s)');
ylabel('u (N)');
title('Control Input');

nexttile;
plot(t, e, 'm', 'LineWidth', 1.4); hold on; grid on;
xlabel('Time (s)');
ylabel('e = r - x_c');
title('Tracking Error');

%% 11) Animation
animate_cart_pendulum(t, x, r, p);


%% ====================== Local Functions ======================

function dx = cartpole_nonlinear(x, u, p)
% Nonlinear cart-pendulum dynamics about upright equilibrium
% x = [theta; theta_dot; x_c; x_c_dot]

theta    = x(1);
thetaDot = x(2);
xcDot    = x(4);

M = p.M;
m = p.m;
l = p.l;
g = p.g;

den = M + m*sin(theta)^2;

thetaDDot = ((M+m)*g*sin(theta) - cos(theta)*(u + m*l*thetaDot^2*sin(theta))) ...
            /(l*den);

xcDDot = (u + m*sin(theta)*(l*thetaDot^2 - g*cos(theta))) / den;

dx = [thetaDot;
      thetaDDot;
      xcDot;
      xcDDot];
end


function xNext = rk4_step(f, x, u, h)
k1 = f(x,             u);
k2 = f(x + 0.5*h*k1,  u);
k3 = f(x + 0.5*h*k2,  u);
k4 = f(x + h*k3,      u);

xNext = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end


function animate_cart_pendulum(t, x, r, p)

theta = x(1,:);
xc    = x(3,:);

cartW = 0.8;
cartH = 0.35;
bobR  = 0.10;

xmin = min([xc, r']) - p.l - 1;
xmax = max([xc, r']) + p.l + 1;
ymin = -0.2;
ymax = cartH + p.l + 0.5;

figure('Name','Animation','Color','w');
axis([xmin xmax ymin ymax]);
axis equal;
grid on;
hold on;

plot([xmin xmax], [0 0], 'k', 'LineWidth', 1.5);   % ground

refLine = xline(r(1), 'm--', 'LineWidth', 1.2);

cartPatch = patch( ...
    [-cartW/2 cartW/2 cartW/2 -cartW/2], ...
    [0 0 cartH cartH], ...
    [0.2 0.6 0.9]);

rodLine = plot([0 0], [cartH cartH+p.l], 'r', 'LineWidth', 2.5);
bobPlot = plot(0, cartH+p.l, 'ko', 'MarkerSize', 14, ...
    'MarkerFaceColor', 'y');

titleHandle = title('Inverted Pendulum Animation');
xlabel('Cart Position (m)');
ylabel('Height (m)');

skip = 5;

for k = 1:skip:length(t)
    th = theta(k);
    xcart = xc(k);

    pivotX = xcart;
    pivotY = cartH;

    bobX = pivotX + p.l*sin(th);
    bobY = pivotY + p.l*cos(th);

    set(cartPatch, 'XData', xcart + [-cartW/2 cartW/2 cartW/2 -cartW/2], ...
                   'YData', [0 0 cartH cartH]);

    set(rodLine, 'XData', [pivotX bobX], ...
                 'YData', [pivotY bobY]);

    set(bobPlot, 'XData', bobX, ...
                 'YData', bobY);

    set(refLine, 'Value', r(k));

    set(titleHandle, 'String', sprintf('t = %.2f s,  x_c = %.3f m,  theta = %.2f deg', ...
    t(k), xcart, th*180/pi));

    drawnow;
end

end