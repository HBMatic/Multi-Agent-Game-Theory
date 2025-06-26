function [attackerTraj, targetTraj, defenderTraj, simTimeVec, capture_info, Ux, Uy] = simulateswitching_rh(attacker_init, target_init, defenders_init, t0, te)
% simulateswitching_rh  Hybrid-mode reference generator over [t0, te]
%   Simulates rescue/intercept trajectories from t0 to te without plotting.

%% 1) Time vectors and parameters
delta       = 0.05;                      % base time step [s]
time_raw    = t0:delta:te;              % coarse time grid
interp_fac  = 12;                        % upsampling factor
delta_int   = delta/ interp_fac;
simTimeVec  = time_raw(1):delta_int:time_raw(end);

%% 2) Initialize state and bookkeeping
num_def     = size(defenders_init,2);
num_agents  = num_def + 2;
xinit       = [attacker_init; target_init; defenders_init(:)];
numState    = numel(xinit);
x_raw       = zeros(numState, numel(time_raw));
x_raw(:,1)  = xinit;

destroyed_defenders = false(1,num_def);
evade_position      = [];
capture             = true;
capture_info        = struct('capture',false,'capture_position',[],'capture_type','');

% initial switching parameters
par = problemData_RH(attacker_init, target_init);

%% 3) Precompute Riccati flows for both modes over [t0, te]
% Intercept mode
cur_par_i = problemData_MD_intercept(destroyed_defenders);
init_i    = initializeHessians(cur_par_i);
[~, Yi]   = ode45(@(t,y) odefun(t,y,cur_par_i,destroyed_defenders), te:-delta:t0, init_i);
Yi        = flip(Yi,1)';

% Rescue mode
cur_par_r = problemData_MD_rescue(destroyed_defenders);
init_r    = initializeHessians(cur_par_r);
[~, Yr]   = ode45(@(t,y) odefun(t,y,cur_par_r,destroyed_defenders), te:-delta:t0, init_r);
Yr        = flip(Yr,1)';

%% 4) Hybrid propagation over coarse grid
total_steps = numel(time_raw);
for idx = 1:total_steps-1
    % update switching parameter
    par = problemData_RH(x_raw(1:2,idx), x_raw(3:4,idx));
    if par.psi <= 0
        mode = 'intercept';
        RiccatiFlow = Yi;
        cur_par     = problemData_MD_intercept(destroyed_defenders);
        capFcn      = @checkCaptureforinterception;
    else
        mode = 'rescue';
        RiccatiFlow = Yr;
        cur_par     = problemData_MD_rescue(destroyed_defenders);
        capFcn      = @checkCaptureforrescue;
    end
    % compute closed-loop A matrix
    Acl = calculateMatrices(RiccatiFlow, idx, cur_par, destroyed_defenders);
    dt  = time_raw(idx+1) - time_raw(idx);
    x_raw(:,idx+1) = expm(Acl * dt) * x_raw(:,idx);

    % extract positions for capture
    Xa = x_raw(1:2, idx+1);
    Xt = x_raw(3:4, idx+1);
    defender_states = x_raw(5:4+2*num_def, idx+1);
    Xd = mat2cell(defender_states, 2, ones(1,num_def));

    % perform capture check
    [cap, pos, ctype, destroyed_defenders, evade_position] = capFcn(Xa, Xd, Xt, cur_par, destroyed_defenders, time_raw, idx, evade_position);
    if ~cap
        capture = false;
        capture_info.capture = true;
        capture_info.capture_position = pos;
        capture_info.capture_type = ctype;
        break;
    end
end

%% 5) Trim arrays to actual length
nSteps = find(~all(x_raw==0),1,'last');
time_raw = time_raw(1:nSteps);
x_raw    = x_raw(:,1:nSteps);

%% 6) Upsample trajectories
numInt = numel(simTimeVec)-1;
x_interp = zeros(numState, numel(simTimeVec));
for i = 1:numState
    x_interp(i,:) = interp1(time_raw, x_raw(i,:), simTimeVec, 'pchip');
end

attackerTraj = x_interp(1:2, :);
targetTraj   = x_interp(3:4, :);
if num_def > 0
    defenderTraj = x_interp(5:4+2*num_def, :);
else
    defenderTraj = [];
end

%% 7) Compute reference velocities
Ux = zeros(num_agents, numInt);
Uy = zeros(num_agents, numInt);
for j = 1:numInt
    dx = (x_interp(:,j+1) - x_interp(:,j)) / delta;
    for a = 1:num_agents
        Ux(a,j) = dx(2*a-1);
        Uy(a,j) = dx(2*a);
    end
end

end % function simulateswitching_rh
