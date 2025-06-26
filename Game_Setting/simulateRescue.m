function [attackerTraj, targetTraj, defenderTraj, time, capture_info, Ux, Uy] = simulateRescue(attacker_init, target_init, defenders_init)
% SIMULATERESCUE Simulates a rescue scenario and returns smooth trajectories and control velocities.

%% --- Parameters ---
T = 20;
delta = 0.05;
fullTime = 0:delta:T;
numSteps = length(fullTime);
interp_factor = 12;  % interpolation factor for smoothing
delta_interp = delta / interp_factor;

%% --- State Initialization ---
xinit = [attacker_init; target_init; defenders_init(:)];
numAttackers = 1;
numTargets = 1;
numDefenders = size(defenders_init, 2);
total_agents = numAttackers + numTargets + numDefenders;
numState = length(xinit);

x = zeros(numState, numSteps);
U = zeros(2 * total_agents, numSteps);
x(:,1) = xinit;
destroyed_defenders = false(1, numDefenders);

%% --- ODE Preprocessing ---
par = problemData_MD_rescue(destroyed_defenders);
init = initializeHessians(par);
[t_temp, y_mat] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init);
y_mat  = flip(y_mat)';
t_temp = flip(t_temp);

%% --- Visualization Setup ---
figure;
hold on;
xlabel('X Position'); ylabel('Y Position');
title('Trajectories of Attacker, Defenders, and Target: Rescue Mode');
axis equal;
[Xa_path, Xd_paths, Xt_path] = initializePlots_2D(xinit, numDefenders);

%% --- Main Simulation Loop ---
capture = true;
k = 1;
c = [];
Ux_raw = zeros(total_agents, numSteps);
Uy_raw = zeros(total_agents, numSteps);

while capture && k < numSteps
    if k == 1
        x(:,1) = xinit;
    end

    c1 = find(destroyed_defenders);
    if ~isequal(c, c1)
        par = problemData_MD_rescue(destroyed_defenders);
        init = initializeHessians(par);
        [t_temp, y_mat] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:fullTime(k), init);
        y_mat  = flip(y_mat)';
        t_temp = flip(t_temp);
    end

    Acl = calculateMatrices(y_mat, k, par, destroyed_defenders);
    if ~isequal(size(Acl), [2 * total_agents, 2 * total_agents])
        rows_to_add = [2*c1+1, 2*(c1+1)];
        rows_to_add = sort(rows_to_add);
        cols_to_add = rows_to_add;
        Acl = expand_matrix_with_zeros(Acl, rows_to_add, cols_to_add);
    end

    if k < numSteps
        dt = fullTime(k+1) - fullTime(k);
    else
        break;
    end

    x(:,k+1) = expm(Acl * dt) * x(:,k);

    % Approximate raw Ux/Uy before smoothing
    u_all = (x(:,k+1) - x(:,k)) / delta;
    for i = 1:total_agents
        Ux_raw(i,k) = u_all(2*i - 1);
        Uy_raw(i,k) = u_all(2*i);
    end

    [Xa, Xd, Xt] = updatePositions_2D(x, k+1, numDefenders, destroyed_defenders);
    updatePlots_2D(Xa_path, Xd_paths, Xt_path, Xa, Xd, Xt, numDefenders, destroyed_defenders);

    [capture, capture_position, capture_type, destroyed_defenders, ~] = ...
        checkCaptureforrescue(Xa, Xd, Xt, par, destroyed_defenders, fullTime, k, []);

    if ~capture
        break;
    end

    k = k + 1;
    c = c1;
    drawnow;
    pause(0.06);
end

%% --- Finalize Raw Output
finalIndex = min(k+1, numSteps);
time_raw = fullTime(1:finalIndex);
x_raw = x(:, 1:finalIndex);
Ux_raw = Ux_raw(:, 1:finalIndex);
Uy_raw = Uy_raw(:, 1:finalIndex);

%% --- Interpolation Step ---
t_interp = time_raw(1):delta_interp:time_raw(end);
x_interp = zeros(numState, length(t_interp));
for i = 1:numState
    x_interp(i,:) = interp1(time_raw, x_raw(i,:), t_interp, 'pchip');
end

%% --- Recompute Smooth Ux/Uy ---
Ux = zeros(total_agents, length(t_interp)-1);
Uy = zeros(total_agents, length(t_interp)-1);

for j = 1:length(t_interp)-1
    dx = (x_interp(:,j+1) - x_interp(:,j)) / delta;
    for i = 1:total_agents
        Ux(i,j) = dx(2*i - 1);
        Uy(i,j) = dx(2*i);
    end
end

%% --- Separate Final Trajectories ---
attackerTraj = x_interp(1:2,:);
targetTraj   = x_interp(3:4,:);
if numDefenders > 0
    defenderTraj = x_interp(5:end,:);
else
    defenderTraj = [];
end

time = t_interp;
capture_info.capture = capture;
capture_info.capture_position = capture_position;
capture_info.capture_type = capture_type;

end
