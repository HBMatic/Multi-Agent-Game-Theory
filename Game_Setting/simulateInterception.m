function [attackerTraj, targetTraj, defenderTraj, time, capture_info] = simulateInterception(attacker_init, target_init, defenders_init)
% simulateInterceptionSeparated Simulates trajectories for an attacker,
% target, and one or more defenders in interception mode.
%
%   [attackerTraj, targetTraj, defenderTraj, time, capture_info] =
%   simulateInterceptionSeparated(attacker_init, target_init, defenders_init)
%   simulates the trajectories given:
%       - attacker_init: a 2x1 vector for the attacker's initial position.
%       - target_init:   a 2x1 vector for the target's initial position.
%       - defenders_init: a 2xN matrix for the initial positions of N defenders.
%
%   OUTPUTS:
%       attackerTraj - 2 x numSteps trajectory for the attacker.
%       targetTraj   - 2 x numSteps trajectory for the target.
%       defenderTraj - (2*N) x numSteps trajectories for the defenders.
%       time         - A vector of time steps.
%       capture_info - A structure with additional simulation outcomes.
%
%   NOTE: This function uses several helper functions (e.g.,
%   problemData_MD_intercept, initializeHessians, odefun, calculateMatrices,
%   expand_matrix_with_zeros, updatePositions_2D, initializePlots_2D,
%   updatePlots_2D, checkCaptureforinterception, displayOutcome_2D) that must
%   be available in your MATLAB path.

    % Simulation parameters
    T = 5;
    delta = 0.05;
    time = 0:delta:T;
    
    % Assemble the initial state vector.
    % The ordering is: attacker (2x1), target (2x1), defenders (2 x numDefenders)
    xinit = [attacker_init; target_init; defenders_init(:)];
    
    % Determine number of agents.
    numAttackers = 1;
    numTargets   = 1;
    numDefenders = size(defenders_init, 2);
    total_agents = numAttackers + numTargets + numDefenders;
    
    % Initialize defender status: false indicates an active (not destroyed) defender.
    destroyed_defenders = false(1, numDefenders);
    
    % Get problem parameters and initial Hessians for ODE solution.
    par = problemData_MD_intercept(destroyed_defenders);
    init = initializeHessians(par);
    
    % Precompute ODE solution for initial setup.
    [t_temp, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init);
    y = flip(y)';  % Now each column corresponds to a time instant.
    t_temp = flip(t_temp);
    
    % Preallocate state trajectory matrix x:
    % x is a (2*total_agents) x numSteps matrix.
    numState = length(xinit);
    numSteps = length(time);
    x = zeros(numState, numSteps);
    x(:,1) = xinit;
    
    % (Optional) Create a figure to display the trajectories.
    figure;
    hold on;
    xlabel('X Position');
    ylabel('Y Position');
    title('Trajectories of Attacker, Defenders, and Target : Interception Mode');
    axis equal;
    
    % Initialize plot handles.
    [Xa_path, Xd_paths, Xt_path] = initializePlots_2D(xinit, numDefenders);
    
    % Simulation loop initialization.
    capture = true;
    k = 1;
    c = [];
    
    while capture
        if k == 1
            x(:,1) = xinit;
        end
        
        % Check for changes in defender status.
        c1 = find(destroyed_defenders);
        if ~isequal(c, c1)
            par = problemData_MD_intercept(destroyed_defenders);
            init = initializeHessians(par);
            [t_temp, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:time(k), init);
            y = flip(y)';
            t_temp = flip(t_temp);
        end
        
        % Calculate the closed-loop system matrix at the current time.
        Acl = calculateMatrices(y, k, par, destroyed_defenders);
        % Ensure Acl has the proper dimensions (2 x total_agents for each agent)
        if ~isequal(size(Acl), size(zeros(2*total_agents)))
            rows_to_add = [2*c1+1, 2*(c1+1)];
            rows_to_add = sort(rows_to_add);
            cols_to_add = [2*c1+1, 2*(c1+1)];
            cols_to_add = sort(cols_to_add);
            Acl = expand_matrix_with_zeros(Acl, rows_to_add, cols_to_add);
        end
        
        % Propagate the state over the current time step.
        x(:,k+1) = expm(Acl * (time(k+1) - time(k))) * x(:,k);
        
        % Update positions based on the current state vector.
        [Xa, Xd, Xt] = updatePositions_2D(x, k+1, numDefenders, destroyed_defenders);
        updatePlots_2D(Xa_path, Xd_paths, Xt_path, Xa, Xd, Xt, numDefenders, destroyed_defenders);
        
        % Check capture conditions and update defender statuses.
        [capture, capture_position, capture_type, destroyed_defenders, ~] = ...
            checkCaptureforinterception(Xa, Xd, Xt, par, destroyed_defenders, time, k, []);
        
        if ~capture
            break;
        end
        
        k = k + 1;
        c = c1;
        drawnow;
        pause(0.06);
    end
    
    % Display final simulation outcome.
    displayOutcome_2D(capture, capture_type, capture_position, par, []);
    
    % Separate trajectories:
    % Attacker trajectory: rows 1-2.
    % Target trajectory: rows 3-4.
    % Defender trajectories: remaining rows (each defender occupies 2 rows).
    attackerTraj = x(1:2, :);
    targetTraj   = x(3:4, :);
    if numDefenders > 0
        defenderTraj = x(5:end, :);
    else
        defenderTraj = [];
    end
    
    % Package additional simulation output.
    capture_info.capture = capture;
    capture_info.capture_position = capture_position;
    capture_info.capture_type = capture_type;
    
end
