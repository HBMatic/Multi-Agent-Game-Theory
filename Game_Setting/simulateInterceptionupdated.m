function [attackerTraj, targetTraj, defenderTraj, time, capture_info] = simulateInterceptionupdated(attacker_init, target_init, defenders_init)
% simulateInterception Simulates trajectories for an attacker, target, and
% one or more defenders in interception mode. If the simulation ends early
% (e.g., due to capture), it truncates the output trajectories accordingly
% instead of filling the remaining entries with zeros.
%
%   [attackerTraj, targetTraj, defenderTraj, time, capture_info] =
%   simulateInterception(attacker_init, target_init, defenders_init)
%   simulates the trajectories given:
%       - attacker_init: 2x1 vector (attacker's initial [x; y])
%       - target_init:   2x1 vector (target's initial [x; y])
%       - defenders_init: 2xN matrix (each column is a defender's [x; y])
%
%   OUTPUTS:
%       attackerTraj - 2 x numSteps trajectory for the attacker.
%       targetTraj   - 2 x numSteps trajectory for the target.
%       defenderTraj - (2*N) x numSteps for the defenders (2 rows per defender).
%       time         - A vector of time steps (truncated if simulation ends early).
%       capture_info - Struct with additional simulation outcomes.

    %% --- Simulation parameters ---
    T = 20;                % Total simulation time
    delta = 0.05;         % Time step
    fullTime = 0:delta:T; % Full time vector (will truncate if capture ends early)
    numSteps = length(fullTime);

    %% --- Assemble the initial state vector ---
    % Ordering: attacker (2x1), target (2x1), defenders (2 x numDefenders)
    xinit = [attacker_init; target_init; defenders_init(:)];

    % Determine number of agents
    numAttackers = 1;
    numTargets   = 1;
    numDefenders = size(defenders_init, 2);
    total_agents = numAttackers + numTargets + numDefenders;

    % Initialize defender status (false => active, not destroyed)
    destroyed_defenders = false(1, numDefenders);

    %% --- ODE-related data ---
    par = problemData_MD_intercept(destroyed_defenders);
    init = initializeHessians(par);

    % Precompute ODE solution for the nominal horizon
    [t_temp, y_mat] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init);
    y_mat  = flip(y_mat)';   % Each column is state at a time instant
    t_temp = flip(t_temp);

    %% --- Prepare for simulation loop ---
    % x will hold the full state trajectory: (2*total_agents) x numSteps
    numState = length(xinit);
    x = zeros(numState, numSteps);
    x(:,1) = xinit;

    % Create a figure (optional) to show the real-time simulation
    figure;
    hold on;
    xlabel('X Position');
    ylabel('Y Position');
    title('Trajectories of Attacker, Defenders, and Target: Interception Mode');
    axis equal;

    % Initialize plot handles (helper function must be on path)
    [Xa_path, Xd_paths, Xt_path] = initializePlots_2D(xinit, numDefenders);

    % Loop variables
    capture = true;
    k = 1;
    c = [];  % Tracks changes in destroyed_defenders

    %% --- Main Simulation Loop ---
    while capture
        if k == 1
            x(:,1) = xinit;
        end

        % If defenders are destroyed mid-simulation, re-init ODE solver
        c1 = find(destroyed_defenders);
        if ~isequal(c, c1)
            par = problemData_MD_intercept(destroyed_defenders);
            init = initializeHessians(par);
            [t_temp, y_mat] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), ...
                                    T:-delta:fullTime(k), init);
            y_mat  = flip(y_mat)';
            t_temp = flip(t_temp);
        end

        % Calculate the closed-loop system matrix at this time step
        Acl = calculateMatrices(y_mat, k, par, destroyed_defenders);

        % Ensure Acl is the correct size (2*total_agents) x (2*total_agents)
        if ~isequal(size(Acl), [2*total_agents, 2*total_agents])
            rows_to_add = [2*c1+1, 2*(c1+1)];
            rows_to_add = sort(rows_to_add);
            cols_to_add = rows_to_add;
            Acl = expand_matrix_with_zeros(Acl, rows_to_add, cols_to_add);
        end

        % Propagate the state over [fullTime(k), fullTime(k+1)]
        if k < numSteps
            dt = fullTime(k+1) - fullTime(k);
        else
            % If somehow we reached the final time step, break
            break;
        end
        x(:, k+1) = expm(Acl * dt) * x(:, k);

        % Update positions for plotting
        [Xa, Xd, Xt] = updatePositions_2D(x, k+1, numDefenders, destroyed_defenders);
        updatePlots_2D(Xa_path, Xd_paths, Xt_path, Xa, Xd, Xt, ...
                       numDefenders, destroyed_defenders);

        % Check for capture
        [capture, capture_position, capture_type, destroyed_defenders, ~] = ...
            checkCaptureforinterception(Xa, Xd, Xt, par, destroyed_defenders, fullTime, k, []);

        if ~capture
            % If capture occurred, break immediately
            break;
        end

        k = k + 1;
        c = c1;
        drawnow;
        pause(0.06);

        % If we've reached the final step without capture, end the loop
        if k >= numSteps
            break;
        end
    end

    %% --- Truncate arrays to the final simulation index ---
    % finalIndex is the last column we wrote to in x
    finalIndex = min(k+1, numSteps);

    % Truncate time and state arrays to avoid trailing zeros
    time = fullTime(1:finalIndex);
    x = x(:, 1:finalIndex);

    % Display final outcome
    displayOutcome_2D(capture, capture_type, capture_position, par, []);

    %% --- Separate the trajectories (attacker, target, defenders) ---
    % Attacker: rows 1-2
    attackerTraj = x(1:2, :);
    % Target: rows 3-4
    targetTraj   = x(3:4, :);
    % Defenders: remaining
    if numDefenders > 0
        defenderTraj = x(5:end, :);
    else
        defenderTraj = [];
    end

    %% --- Package additional output info ---
    capture_info.capture           = capture;
    capture_info.capture_position  = capture_position;
    capture_info.capture_type      = capture_type;

end
