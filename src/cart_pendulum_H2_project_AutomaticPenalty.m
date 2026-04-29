%% Inverted Pendulum on a Cart - Practical H2/LQG Project Version
% State definition:
% x = [theta; theta_dot; x_c; x_c_dot]
% theta = 0 means upright pendulum.

clear; clc; close all;

%% 1) Physical parameters
p.M  = 2;          % cart mass [kg]
p.m  = 1;          % pendulum mass [kg]
p.l  = 4;          % pendulum length [m]
p.g  = 9.8;        % gravity [m/s^2]

% Practical nonidealities
p.bCart = 0.25;    % cart viscous friction coefficient [N/(m/s)]

%% 2) Simulation settings
dt   = 0.005;      % controller sampling time [s]
tEnd = 30;         % total simulation time [s]
t    = (0:dt:tEnd).';
N    = numel(t);

% Safety and actuator limits
uMax       = 20;       % maximum commanded force [N]
duMax      = 250;      % maximum force slew rate [N/s]
tauAct     = 0.04;     % actuator first-order time constant [s]
trackLimit = 2.0;      % practical cart travel limit [m]
thetaLimit = 30;       % safety angle limit [deg]

% Animation/video options
runAnimation = true;
saveVideo    = true;  % set true to export MP4
videoName    = 'cart_pendulum_animation.mp4';

%% 3) Linearized model about upright equilibrium
% The controller and observer are designed from the linear model.
% The actual validation simulation below uses the nonlinear plant.

A = [ 0                              1   0   0;
      ((p.M+p.m)*p.g)/(p.M*p.l)      0   0   0;
      0                              0   0   1;
     -(p.m*p.g)/p.M                  0   0   0 ];

B = [0;
    -1/(p.M*p.l);
     0;
     1/p.M];

% Measured outputs:
% theta and x_c are measured. Velocities are estimated.
C = [1 0 0 0;
     0 0 1 0];

Ctrack = [0 0 1 0];

fprintf('rank(ctrb(A,B)) = %d\n', rank(ctrb(A,B)));
fprintf('rank(obsv(A,C)) = %d\n', rank(obsv(A,C)));

%% 4) Kalman/LQE observer design
% LQG/H2-style structure:
% LQR gives the state-feedback gain, and LQE gives the estimator gain.

G  = eye(4);

Qn = diag([1e-4 2e-2 1e-4 2e-2]);   % process noise covariance
Rn = diag([5e-4 8e-4]);             % measurement noise covariance

Lobs = lqe(A, G, C, Qn, Rn);

disp('Observer gain Lobs = ');
disp(Lobs);

%% 5) Servo LQR design
% Add integral state xi = integral(r - x_c) to improve position tracking.

Aaug = [A,            zeros(4,1);
       -Ctrack,       0          ];
Baug = [B;
        0];

% Tuning notes:
% - state order is [theta theta_dot x_c x_c_dot]
% - increase Qservo(1,1) and Qservo(2,2) to protect pendulum angle
% - increase Qservo(3,3) and Qservo(5,5) to improve cart tracking
% - increase Rservo to reduce control effort
manualStateWeights = [120 15 260 150];  % [theta theta_dot x_c x_c_dot]
integralWeight     = 250;
Rservo = 1.2;

% Set false if you want to rerun the script with the manual values above.
optimizeStateWeights = true;

if optimizeStateWeights
    optCases = make_weight_optimization_cases();
    [stateWeights, bestWeightCost] = optimize_servo_state_weights( ...
        manualStateWeights, integralWeight, Rservo, ...
        Aaug, Baug, A, B, C, Lobs, p, ...
        dt, tEnd, uMax, duMax, tauAct, trackLimit, thetaLimit, optCases);

    optimizedWeightTable = table( ...
        {'theta'; 'theta_dot'; 'x_c'; 'x_c_dot'}, stateWeights(:), ...
        'VariableNames', {'State','OptimizedWeight'});

    disp('Optimized state weights:');
    disp(optimizedWeightTable);
    fprintf('Optimized state-weight vector [theta theta_dot x_c x_c_dot] = [%.6g %.6g %.6g %.6g]\n', stateWeights);
    fprintf('Best multi-case nonlinear objective cost = %.6g\n', bestWeightCost);
    writetable(optimizedWeightTable, 'optimized_servo_state_weights.csv');
    save('optimized_servo_state_weights.mat', 'stateWeights', 'bestWeightCost');
else
    stateWeights = manualStateWeights;
end

Qservo = diag([stateWeights integralWeight]);

disp('Final Qservo used by LQR = ');
disp(Qservo);

Kaug = lqr(Aaug, Baug, Qservo, Rservo);
Kx = Kaug(1:4);
Ki = Kaug(5);

disp('State feedback gain Kx = ');
disp(Kx);
disp('Integral gain Ki = ');
disp(Ki);

%% 6) Reference, disturbance, and sensor noise

% Smooth reference is more practical than ideal step commands.
% It avoids asking the actuator to create an unrealistic instantaneous motion.
r = smooth_reference(t);

% External disturbance force acting on the cart.
% Example: a short push at t = 6 s, and another opposite push at t = 12 s.
d = zeros(N,1);
d(t >= 6.0  & t <= 6.20)  =  1.5;
d(t >= 12.0 & t <= 12.15) = -1.2;

% Measurement noise
rng(1);  % reproducible simulation
thetaNoiseStd = 0.002;   % rad
xcNoiseStd    = 0.003;   % m
noise = [thetaNoiseStd*randn(N,1), xcNoiseStd*randn(N,1)];

%% 7) Initialization and data storage

x    = zeros(4,N);    % true nonlinear states
xhat = zeros(4,N);    % observer states
y    = zeros(2,N);    % noisy measurements

uCmdUnsat = zeros(N,1);  % raw controller command
uCmdSat   = zeros(N,1);  % saturated command
uAct      = zeros(N,1);  % actual actuator force after lag/rate limit

e  = zeros(N,1);     % tracking error
xi = zeros(N,1);     % integral state

% Initial condition
x(:,1)    = [5*pi/180; 0; 0; 0];  % 5 deg initial pendulum error
xhat(:,1) = [0; 0; 0; 0];
xi(1)     = 0;

safetyStop = false;
stopIndex  = N;

%% 8) Main digital-control simulation loop

for k = 1:N-1

    % Sensor measurement
    y(:,k) = C*x(:,k) + noise(k,:)';

    % Tracking error using measured cart position
    e(k) = r(k) - y(2,k);

    % Observer-based servo control
    % Negative sign because MATLAB lqr returns positive K for u = -Kx.
    uCmdUnsat(k) = -Kx*xhat(:,k) - Ki*xi(k);

    % Force saturation
    uCmdSat(k) = saturate(uCmdUnsat(k), -uMax, uMax);

    % Anti-windup:
    % Integrate only if actuator command is not saturated too hard.
    % This helps prevent excessive integral buildup.
    if abs(uCmdUnsat(k) - uCmdSat(k)) < 1e-9
        xi(k+1) = xi(k) + dt*e(k);
    else
        xi(k+1) = xi(k);
    end

    % Actuator rate limit + first-order lag
    uDesiredDot = (uCmdSat(k) - uAct(k))/tauAct;
    uLimitedDot = saturate(uDesiredDot, -duMax, duMax);
    uAct(k+1) = uAct(k) + dt*uLimitedDot;

    % Nonlinear plant update:
    % use actual actuator force plus external disturbance
    uPlant = uAct(k) + d(k);
    x(:,k+1) = rk4_step(@(xs,uu) cartpole_nonlinear_practical(xs,uu,p), ...
                        x(:,k), uPlant, dt);

    % Linear observer update using actual actuator force estimate
    xhatDot = A*xhat(:,k) + B*uAct(k) + Lobs*(y(:,k) - C*xhat(:,k));
    xhat(:,k+1) = xhat(:,k) + dt*xhatDot;

    % Safety check
    if abs(x(1,k+1))*180/pi > thetaLimit || abs(x(3,k+1)) > trackLimit
        safetyStop = true;
        stopIndex = k+1;

        % Fill the remaining arrays with the last valid values.
        x(:,k+2:end)       = repmat(x(:,k+1), 1, N-k-1);
        xhat(:,k+2:end)    = repmat(xhat(:,k+1), 1, N-k-1);
        xi(k+2:end)        = xi(k+1);
        uAct(k+2:end)      = uAct(k+1);
        uCmdSat(k+1:end)   = uCmdSat(k);
        uCmdUnsat(k+1:end) = uCmdUnsat(k);
        break;
    end
end

% Last sample
y(:,N) = C*x(:,N) + noise(N,:)';
e(N)   = r(N) - y(2,N);
if N > 1
    uCmdUnsat(N) = uCmdUnsat(N-1);
    uCmdSat(N)   = uCmdSat(N-1);
end

%% 9) Performance summary

thetaDeg = x(1,:)'*180/pi;
xc       = x(3,:)';
xcErr    = r - xc;

maxThetaDeg = max(abs(thetaDeg));
maxUCmd     = max(abs(uCmdSat));
maxUAct     = max(abs(uAct));
rmseTrack   = sqrt(mean(xcErr.^2));
finalError  = xcErr(end);

thetaEstErrDeg = (x(1,:) - xhat(1,:))'*180/pi;
xcEstErr        = (x(3,:) - xhat(3,:))';

rmsThetaEstErrDeg = sqrt(mean(thetaEstErrDeg.^2));
rmsXcEstErr        = sqrt(mean(xcEstErr.^2));

satPercent = 100*sum(abs(uCmdUnsat) > uMax)/N;

settlingTol = 0.03;  % [m]
settlingTime = estimate_settling_time(t, xcErr, settlingTol);

recoveryAfterDist1 = estimate_recovery_time(t, thetaDeg, xcErr, 6.20,  3, 0.05);
recoveryAfterDist2 = estimate_recovery_time(t, thetaDeg, xcErr, 12.15, 3, 0.05);

fprintf('\n================ Practical Project Performance Summary ================\n');
fprintf('Max |theta|                    = %.4f deg\n', maxThetaDeg);
fprintf('Max |commanded saturated force| = %.4f N\n', maxUCmd);
fprintf('Max |actual actuator force|     = %.4f N\n', maxUAct);
fprintf('Force saturation percentage     = %.2f %%\n', satPercent);
fprintf('Tracking RMSE for x_c           = %.6f m\n', rmseTrack);
fprintf('Final cart tracking error       = %.6f m\n', finalError);
fprintf('Settling time into +/- %.3f m    = %.4f s\n', settlingTol, settlingTime);
fprintf('RMS theta estimation error       = %.6f deg\n', rmsThetaEstErrDeg);
fprintf('RMS x_c estimation error         = %.6f m\n', rmsXcEstErr);

if isnan(recoveryAfterDist1)
    fprintf('Recovery after disturbance 1     = not recovered by end of simulation\n');
else
    fprintf('Recovery after disturbance 1     = %.4f s after disturbance ends\n', recoveryAfterDist1);
end

if isnan(recoveryAfterDist2)
    fprintf('Recovery after disturbance 2     = not recovered by end of simulation\n');
else
    fprintf('Recovery after disturbance 2     = %.4f s after disturbance ends\n', recoveryAfterDist2);
end

if safetyStop
    fprintf('Safety stop triggered at t = %.4f s\n', t(stopIndex));
else
    fprintf('Safety stop: no violation\n');
end

%% 10) Save log data

logTable = table( ...
    t, ...
    x(1,:)', x(2,:)', x(3,:)', x(4,:)', ...
    xhat(1,:)', xhat(2,:)', xhat(3,:)', xhat(4,:)', ...
    y(1,:)', y(2,:)', ...
    r, e, uCmdUnsat, uCmdSat, uAct, d, ...
    thetaEstErrDeg, xcEstErr, ...
    'VariableNames', { ...
    't', ...
    'theta', 'theta_dot', 'xc', 'xc_dot', ...
    'theta_hat', 'theta_dot_hat', 'xc_hat', 'xc_dot_hat', ...
    'theta_meas', 'xc_meas', ...
    'xc_ref', 'tracking_error', ...
    'u_cmd_unsat', 'u_cmd_sat', 'u_actual', 'disturbance', ...
    'theta_est_error_deg', 'xc_est_error'});

writetable(logTable, 'cart_pendulum_practical_log.csv');
disp('Log data saved to cart_pendulum_practical_log.csv');

%% 11) Dashboard plots

figure('Name','Practical Inverted Pendulum H2/LQG Project','Color','w');
tiledlayout(4,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t, thetaDeg, 'b', 'LineWidth', 1.4); hold on; grid on;
plot(t, xhat(1,:)'*180/pi, 'b--', 'LineWidth', 1.1);
yline(thetaLimit, 'k--');
yline(-thetaLimit, 'k--');
ylabel('\theta (deg)');
legend('\theta', '\theta hat', 'safety limit', 'Location', 'best');
title('Pendulum Angle');

nexttile;
plot(t, x(2,:)', 'b', 'LineWidth', 1.4); hold on; grid on;
plot(t, xhat(2,:)', 'b--', 'LineWidth', 1.1);
ylabel('\theta dot (rad/s)');
legend('\theta dot', '\theta dot hat', 'Location', 'best');
title('Angular Velocity');

nexttile;
plot(t, xc, 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, xhat(3,:)', 'r--', 'LineWidth', 1.1);
plot(t, r, 'k:', 'LineWidth', 1.5);
yline(trackLimit, 'k--');
yline(-trackLimit, 'k--');
ylabel('x_c (m)');
legend('x_c', 'x_c hat', 'reference', 'track limit', 'Location', 'best');
title('Cart Position Tracking');

nexttile;
plot(t, x(4,:)', 'r', 'LineWidth', 1.4); hold on; grid on;
plot(t, xhat(4,:)', 'r--', 'LineWidth', 1.1);
ylabel('x_c dot (m/s)');
legend('x_c dot', 'x_c dot hat', 'Location', 'best');
title('Cart Velocity');

nexttile;
plot(t, uCmdUnsat, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0); hold on; grid on;
plot(t, uCmdSat, 'g--', 'LineWidth', 1.2);
plot(t, uAct, 'g', 'LineWidth', 1.5);
yline(uMax, 'k--');
yline(-uMax, 'k--');
ylabel('Force (N)');
legend('raw command', 'saturated command', 'actual actuator force', 'Location', 'best');
title('Actuator Limitation');

nexttile;
plot(t, d, 'm', 'LineWidth', 1.4); grid on;
ylabel('d (N)');
title('External Disturbance');

nexttile;
plot(t, xcErr, 'm', 'LineWidth', 1.4); hold on; grid on;
yline(settlingTol, 'k--');
yline(-settlingTol, 'k--');
xlabel('Time (s)');
ylabel('r - x_c (m)');
title('Tracking Error');

nexttile;
plot(t, thetaEstErrDeg, 'b', 'LineWidth', 1.2); hold on; grid on;
plot(t, xcEstErr, 'r', 'LineWidth', 1.2);
xlabel('Time (s)');
legend('\theta error (deg)', 'x_c error (m)', 'Location', 'best');
title('Observer Estimation Error');

%% 12) Animation

if runAnimation
    animate_cart_pendulum_practical(t, x, r, uAct, d, p, trackLimit, ...
                                    saveVideo, videoName);
end


%% ====================== Local Functions ======================

function cases = make_weight_optimization_cases()
% Multi-case set used by patternsearch. These cases keep the objective from
% overfitting one initial condition or one disturbance profile.

cases = struct( ...
    'name', {}, ...
    'x0', {}, ...
    'refScale', {}, ...
    'disturbanceScale', {}, ...
    'thetaNoiseStd', {}, ...
    'xcNoiseStd', {}, ...
    'noiseSeed', {}, ...
    'caseWeight', {});

cases(1).name = 'nominal';
cases(1).x0 = [5*pi/180; 0; 0; 0];
cases(1).refScale = 1.0;
cases(1).disturbanceScale = 1.0;
cases(1).thetaNoiseStd = 0.002;
cases(1).xcNoiseStd = 0.003;
cases(1).noiseSeed = 11;
cases(1).caseWeight = 1.0;

cases(2).name = 'larger_angle_and_push';
cases(2).x0 = [10*pi/180; 0; 0; 0];
cases(2).refScale = 1.0;
cases(2).disturbanceScale = 1.35;
cases(2).thetaNoiseStd = 0.002;
cases(2).xcNoiseStd = 0.003;
cases(2).noiseSeed = 12;
cases(2).caseWeight = 1.25;

cases(3).name = 'cart_offset';
cases(3).x0 = [-5*pi/180; 0; 0.15; 0];
cases(3).refScale = 0.85;
cases(3).disturbanceScale = 0.9;
cases(3).thetaNoiseStd = 0.0025;
cases(3).xcNoiseStd = 0.0035;
cases(3).noiseSeed = 13;
cases(3).caseWeight = 1.0;

cases(4).name = 'moving_cart';
cases(4).x0 = [3*pi/180; 0; -0.10; 0.15];
cases(4).refScale = 1.15;
cases(4).disturbanceScale = 1.1;
cases(4).thetaNoiseStd = 0.002;
cases(4).xcNoiseStd = 0.003;
cases(4).noiseSeed = 14;
cases(4).caseWeight = 1.0;

end


function [bestStateWeights, bestCost] = optimize_servo_state_weights( ...
    initialStateWeights, integralWeight, Rservo, ...
    Aaug, Baug, A, B, C, Lobs, p, ...
    dt, tEnd, uMax, duMax, tauAct, trackLimit, thetaLimit, optCases)
% Optimize positive LQR state weights in log10-space.
% MATLAB's patternsearch is used when the Global Optimization Toolbox exists.
% Otherwise a small local pattern-search fallback is used.

x0 = log10(initialStateWeights(:));
lb = log10([1e-2; 1e-2; 1e-2; 1e-2]);
ub = log10([1e5; 1e5; 1e5; 1e5]);

objective = @(z) multi_case_servo_cost( ...
    10.^z(:).', integralWeight, Rservo, ...
    Aaug, Baug, A, B, C, Lobs, p, ...
    dt, tEnd, uMax, duMax, tauAct, trackLimit, thetaLimit, optCases);

fprintf('\nStarting state-weight search over log10([theta theta_dot x_c x_c_dot])...\n');

if exist('patternsearch', 'file') == 2
    psOptions = optimoptions('patternsearch', ...
        'Display', 'iter', ...
        'MaxIterations', 40, ...
        'MaxFunctionEvaluations', 250, ...
        'UseParallel', false);
    [zBest, bestCost] = patternsearch(objective, x0, [], [], [], [], lb, ub, [], psOptions);
else
    warning(['MATLAB patternsearch was not found. Using the local ', ...
             'pattern-search fallback in this file instead.']);
    [zBest, bestCost] = local_pattern_search(objective, x0, lb, ub, 40, 250);
end

bestStateWeights = 10.^zBest(:).';

end


function [xBest, fBest] = local_pattern_search(objective, x0, lb, ub, maxIter, maxEval)
% Minimal coordinate pattern-search fallback for installations without
% Global Optimization Toolbox. Variables are expected to be in log-space.

xBest = min(max(x0(:), lb(:)), ub(:));
fBest = objective(xBest);
meshSize = 0.5;
meshTol = 1e-2;
evalCount = 1;

for iter = 1:maxIter
    improved = false;
    directions = [eye(numel(xBest)), -eye(numel(xBest))];

    for iDir = 1:size(directions,2)
        if evalCount >= maxEval
            return;
        end

        xTrial = xBest + meshSize*directions(:,iDir);
        xTrial = min(max(xTrial, lb(:)), ub(:));
        fTrial = objective(xTrial);
        evalCount = evalCount + 1;

        if fTrial < fBest
            xBest = xTrial;
            fBest = fTrial;
            improved = true;
            fprintf('local patternsearch iter %d eval %d cost %.6g\n', ...
                    iter, evalCount, fBest);
        end
    end

    if improved
        meshSize = min(1.0, 1.2*meshSize);
    else
        meshSize = 0.5*meshSize;
        if meshSize < meshTol
            return;
        end
    end
end

end


function J = multi_case_servo_cost( ...
    stateWeights, integralWeight, Rservo, ...
    Aaug, Baug, A, B, C, Lobs, p, ...
    dt, tEnd, uMax, duMax, tauAct, trackLimit, thetaLimit, optCases)
% Fixed objective used to judge candidate LQR weights.
% The searched Q weights are controller parameters, not the cost definition.

if any(~isfinite(stateWeights)) || any(stateWeights <= 0)
    J = 1e12;
    return;
end

try
    Qservo = diag([stateWeights integralWeight]);
    Kaug = lqr(Aaug, Baug, Qservo, Rservo);
catch
    J = 1e12;
    return;
end

Kx = Kaug(1:4);
Ki = Kaug(5);

J = 0;
for iCase = 1:numel(optCases)
    sim = simulate_cart_pendulum_case( ...
        Kx, Ki, A, B, C, Lobs, p, ...
        dt, tEnd, uMax, duMax, tauAct, trackLimit, thetaLimit, optCases(iCase));

    Jcase = score_cart_pendulum_case(sim, uMax, trackLimit, thetaLimit);
    J = J + optCases(iCase).caseWeight*Jcase;
end

if ~isfinite(J)
    J = 1e12;
end

end


function sim = simulate_cart_pendulum_case( ...
    Kx, Ki, A, B, C, Lobs, p, ...
    dt, tEnd, uMax, duMax, tauAct, trackLimit, thetaLimit, caseCfg)
% Nonlinear closed-loop simulation used inside the optimizer.

t = (0:dt:tEnd).';
N = numel(t);

r = caseCfg.refScale*smooth_reference(t);
d = zeros(N,1);
d(t >= 6.0  & t <= 6.20)  =  1.5*caseCfg.disturbanceScale;
d(t >= 12.0 & t <= 12.15) = -1.2*caseCfg.disturbanceScale;

rng(caseCfg.noiseSeed);
noise = [caseCfg.thetaNoiseStd*randn(N,1), caseCfg.xcNoiseStd*randn(N,1)];

x    = zeros(4,N);
xhat = zeros(4,N);
y    = zeros(2,N);

uCmdUnsat = zeros(N,1);
uCmdSat   = zeros(N,1);
uAct      = zeros(N,1);

e  = zeros(N,1);
xi = zeros(N,1);

x(:,1)    = caseCfg.x0(:);
xhat(:,1) = [0; 0; 0; 0];

safetyStop = false;
stopIndex  = N;

for k = 1:N-1
    y(:,k) = C*x(:,k) + noise(k,:)';
    e(k) = r(k) - y(2,k);

    uCmdUnsat(k) = -Kx*xhat(:,k) - Ki*xi(k);
    uCmdSat(k) = saturate(uCmdUnsat(k), -uMax, uMax);

    if abs(uCmdUnsat(k) - uCmdSat(k)) < 1e-9
        xi(k+1) = xi(k) + dt*e(k);
    else
        xi(k+1) = xi(k);
    end

    uDesiredDot = (uCmdSat(k) - uAct(k))/tauAct;
    uLimitedDot = saturate(uDesiredDot, -duMax, duMax);
    uAct(k+1) = uAct(k) + dt*uLimitedDot;

    uPlant = uAct(k) + d(k);
    x(:,k+1) = rk4_step(@(xs,uu) cartpole_nonlinear_practical(xs,uu,p), ...
                        x(:,k), uPlant, dt);

    xhatDot = A*xhat(:,k) + B*uAct(k) + Lobs*(y(:,k) - C*xhat(:,k));
    xhat(:,k+1) = xhat(:,k) + dt*xhatDot;

    if abs(x(1,k+1))*180/pi > thetaLimit || abs(x(3,k+1)) > trackLimit
        safetyStop = true;
        stopIndex = k+1;

        x(:,k+2:end)       = repmat(x(:,k+1), 1, N-k-1);
        uAct(k+2:end)      = uAct(k+1);
        uCmdUnsat(k+1:end) = uCmdUnsat(k);
        break;
    end
end

y(:,N) = C*x(:,N) + noise(N,:)';
e(N) = r(N) - y(2,N);
if N > 1
    uCmdUnsat(N) = uCmdUnsat(N-1);
end

sim.t = t;
sim.x = x;
sim.r = r;
sim.e = e;
sim.uAct = uAct;
sim.uCmdUnsat = uCmdUnsat;
sim.safetyStop = safetyStop;
sim.stopIndex = stopIndex;

end


function J = score_cart_pendulum_case(sim, uMax, trackLimit, thetaLimit)
% Dimensionless score for nonlinear performance.

t = sim.t;
theta = sim.x(1,:)';
thetaDot = sim.x(2,:)';
xc = sim.x(3,:)';
xcDot = sim.x(4,:)';
xcErr = sim.r - xc;
u = sim.uAct;

thetaScale = 8*pi/180;
thetaDotScale = 0.8;
xcErrScale = 0.06;
xcDotScale = 0.35;

runningCost = ...
    6.0*(theta/thetaScale).^2 + ...
    0.8*(thetaDot/thetaDotScale).^2 + ...
    4.5*(xcErr/xcErrScale).^2 + ...
    0.7*(xcDot/xcDotScale).^2 + ...
    0.12*(u/uMax).^2;

J = trapz(t, runningCost)/max(t(end), eps);

maxThetaDeg = max(abs(theta))*180/pi;
maxCart = max(abs(xc));
satPercent = 100*sum(abs(sim.uCmdUnsat) > uMax)/numel(t);
finalError = abs(xcErr(end));
settlingTime = estimate_settling_time(t, xcErr, 0.03);

J = J + 2.0*(finalError/0.03)^2;
J = J + 0.04*satPercent;
J = J + 20.0*max(0, maxThetaDeg/(0.65*thetaLimit) - 1)^2;
J = J + 15.0*max(0, maxCart/(0.85*trackLimit) - 1)^2;

if isinf(settlingTime)
    J = J + 100;
else
    J = J + 0.25*settlingTime;
end

if sim.safetyStop
    J = J + 1e6 + 1e5*(t(end) - t(sim.stopIndex))/max(t(end), eps);
end

end


function r = smooth_reference(t)
% Smooth cart reference trajectory.
% The reference moves between several target positions using cubic blending.

r = zeros(size(t));

for k = 1:numel(t)
    tk = t(k);

    if tk < 2
        r(k) = 0;
    elseif tk < 5
        r(k) = cubic_blend(tk, 2, 5, 0, 0.5);
    elseif tk < 8
        r(k) = 0.5;
    elseif tk < 11
        r(k) = cubic_blend(tk, 8, 11, 0.5, -0.4);
    elseif tk < 14
        r(k) = -0.4;
    elseif tk < 17
        r(k) = cubic_blend(tk, 14, 17, -0.4, 0);
    else
        r(k) = 0;
    end
end

end


function y = cubic_blend(t, t0, tf, y0, yf)
% Cubic smoothstep interpolation between y0 and yf.

s = (t - t0)/(tf - t0);
s = min(max(s,0),1);
sigma = 3*s^2 - 2*s^3;

y = y0 + (yf - y0)*sigma;

end


function dx = cartpole_nonlinear_practical(x, u, p)
% Nonlinear inverted-pendulum-on-cart dynamics.
% Includes cart viscous friction through the net horizontal force.
%
% x = [theta; theta_dot; x_c; x_c_dot]
% theta = 0 is upright.

theta    = x(1);
thetaDot = x(2);
xcDot    = x(4);

M = p.M;
m = p.m;
l = p.l;
g = p.g;

% Net force applied to cart after actuator force and cart friction
F = u - p.bCart*xcDot;

den = M + m*sin(theta)^2;

thetaDDot = ((M+m)*g*sin(theta) - cos(theta)*(F + m*l*thetaDot^2*sin(theta))) ...
            /(l*den);

xcDDot = (F + m*sin(theta)*(l*thetaDot^2 - g*cos(theta))) / den;

dx = [thetaDot;
      thetaDDot;
      xcDot;
      xcDDot];

end


function xNext = rk4_step(f, x, u, h)
% Fourth-order Runge-Kutta integration step.

k1 = f(x,             u);
k2 = f(x + 0.5*h*k1,  u);
k3 = f(x + 0.5*h*k2,  u);
k4 = f(x + h*k3,      u);

xNext = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

end


function y = saturate(x, xmin, xmax)
% Saturate scalar input.

y = min(max(x, xmin), xmax);

end


function Ts = estimate_settling_time(t, err, tol)
% Estimate the first time after which the absolute error remains within tol.

idxLastOut = find(abs(err) > tol, 1, 'last');

if isempty(idxLastOut)
    Ts = 0;
elseif idxLastOut == numel(t)
    Ts = inf;
else
    Ts = t(idxLastOut + 1);
end

end


function Tr = estimate_recovery_time(t, thetaDeg, err, tDistEnd, thetaTolDeg, errTol)
% Estimate recovery time after disturbance ends.
% Recovered means both angle and tracking error remain inside tolerances.

idxStart = find(t >= tDistEnd, 1, 'first');
Tr = NaN;

if isempty(idxStart)
    return;
end

for k = idxStart:numel(t)
    condition = abs(thetaDeg(k:end)) <= thetaTolDeg & abs(err(k:end)) <= errTol;
    if all(condition)
        Tr = t(k) - tDistEnd;
        return;
    end
end

end


function animate_cart_pendulum_practical(t, x, r, uAct, d, p, trackLimit, saveVideo, videoName)

theta = x(1,:);
xc    = x(3,:);

cartW = 0.8;
cartH = 0.35;

xmin = -trackLimit - p.l - 0.5;
xmax =  trackLimit + p.l + 0.5;
ymin = -0.25;
ymax = cartH + p.l + 0.75;

fig = figure('Name','Practical Cart-Pendulum Animation','Color','w');
axis([xmin xmax ymin ymax]);
axis equal;
grid on;
hold on;

plot([xmin xmax], [0 0], 'k', 'LineWidth', 1.5);   % ground
xline(-trackLimit, 'k--', 'LineWidth', 1.0);
xline( trackLimit, 'k--', 'LineWidth', 1.0);

refLine = xline(r(1), 'm--', 'LineWidth', 1.4);

cartPatch = patch( ...
    [-cartW/2 cartW/2 cartW/2 -cartW/2], ...
    [0 0 cartH cartH], ...
    [0.2 0.6 0.9]);

rodLine = plot([0 0], [cartH cartH+p.l], 'r', 'LineWidth', 2.5);
bobPlot = plot(0, cartH+p.l, 'ko', 'MarkerSize', 14, ...
    'MarkerFaceColor', 'y');

forceArrow = quiver(0, cartH/2, 0, 0, 0, ...
    'LineWidth', 1.8, 'MaxHeadSize', 1.8);

titleHandle = title('Practical Inverted Pendulum Animation');
xlabel('Cart Position (m)');
ylabel('Height (m)');

if saveVideo
    vw = VideoWriter(videoName, 'MPEG-4');
    vw.FrameRate = 30;
    open(vw);
end

skip = max(1, round(0.025/(t(2)-t(1))));  % about 40 FPS animation

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

    % Force arrow, scaled for visibility
    arrowScale = 0.08;
    set(forceArrow, 'XData', xcart, ...
                    'YData', cartH/2, ...
                    'UData', arrowScale*uAct(k), ...
                    'VData', 0);

    set(titleHandle, 'String', sprintf( ...
        't = %.2f s | x_c = %.3f m | ref = %.3f m | theta = %.2f deg | u = %.2f N | d = %.2f N', ...
        t(k), xcart, r(k), th*180/pi, uAct(k), d(k)));

    drawnow;

    if saveVideo
        writeVideo(vw, getframe(fig));
    end
end

if saveVideo
    close(vw);
    fprintf('Animation video saved to %s\n', videoName);
end

end
