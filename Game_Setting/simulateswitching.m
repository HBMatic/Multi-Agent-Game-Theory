function [attackerTraj, targetTraj, defenderTraj, simTimeVec, capture_info, Ux, Uy, mdata ] = simulateswitching(attacker_init, target_init, defenders_init)
% Hybrid mode simulation: live plot, mode log, outcome, and reference output

T = 20;
delta = 0.05;
time = 0:delta:T;
interp_factor = 12;
delta_interp = delta / interp_factor;
defender = size(defenders_init, 2);
agents = defender + 2;
destroyed_defenders = false(1, defender);
evade_position = [];
xinit = [attacker_init; target_init; defenders_init(:)];
numState = length(xinit);
maxSteps = length(time);

x = zeros(numState, maxSteps); x(:,1) = xinit;
k = 1; c = [];
capture = true; mdata = string([]);
capture_position = []; capture_type = [];

% -- Plot setup using your updatePlots_2D structure
figure; hold on; axis equal; grid on;
xlabel('X Position'); ylabel('Y Position');
parr = problemData_MD_rescue(destroyed_defenders);
pari = problemData_MD_intercept(destroyed_defenders);
txt = {['Trajectories (\mu =',num2str(parr.meu),',\lambda =',num2str(pari.lamda),') '],': RH Mode (\sigma_a > \sigma_d_i)'};
title(txt);

[Xa_path, Xd_paths, Xt_path] = initializePlots_2D(xinit, defender);

t0 = 0;
par = problemData_RH(xinit(1:2), xinit(2*(defender+1)+1:2*(defender+2)));
te = t0 + par.deltaRH;

% Pre-initialize ODE outputs
pari = problemData_MD_intercept(destroyed_defenders);
initi = initializeHessians(pari);
[~, yi] = ode45(@(t, y) odefun(t, y, pari, destroyed_defenders), T:-delta:0, initi); yi = flip(yi)';
parr = problemData_MD_rescue(destroyed_defenders);
initr = initializeHessians(parr);
[~, yr] = ode45(@(t, y) odefun(t, y, parr, destroyed_defenders), T:-delta:0, initr); yr = flip(yr)';

while capture && te <= T
    if par.psi <= 0, mode = 'intercept'; else, mode = 'rescue'; end
    mdata = [mdata; string(mode)];
    if strcmp(mode, 'intercept')
        pari = problemData_MD_intercept(destroyed_defenders);
        initi = initializeHessians(pari);
        [~, yi] = ode45(@(t, y) odefun(t, y, pari, destroyed_defenders), T:-delta:0, initi); yi = flip(yi)';
        for j = k:k+length(t0:delta:te)-2
            if j == 1, x(:,j) = xinit; end
            if j+1 <= length(time)
                c1 = find(destroyed_defenders);
                if ~isequal(c, c1)
                    pari = problemData_MD_intercept(destroyed_defenders);
                    initi = initializeHessians(pari);
                    [~, yi] = ode45(@(t, y) odefun(t, y, pari, destroyed_defenders), T:-delta:0, initi); yi = flip(yi)';
                end
                Acli = calculateMatrices(yi, j, pari, destroyed_defenders);
                if size(Acli,1) ~= 2*agents
                    rows_to_add = [2*c1+1,2*(c1+1)]; rows_to_add = sort(rows_to_add);
                    cols_to_add = rows_to_add; Acli = expand_matrix_with_zeros(Acli,rows_to_add,cols_to_add);
                end
                x(:,j+1) = expm(Acli * (time(j+1) - time(j))) * x(:,j);
                [Xai, Xdi, Xti] = updatePositions_2D(x, j+1, defender, destroyed_defenders);
                updatePlots_2D(Xa_path, Xd_paths, Xt_path, Xai, Xdi, Xti, defender, destroyed_defenders);
                [capture, capture_position, capture_type, destroyed_defenders, evade_position] = ...
                    checkCaptureforinterception(Xai, Xdi, Xti, pari, destroyed_defenders, time, j, evade_position);
                drawnow; pause(0.06);
            end
            if ~capture, break; end
            c = c1;
        end
        k = j + 1;
    else
        parr = problemData_MD_rescue(destroyed_defenders);
        initr = initializeHessians(parr);
        [~, yr] = ode45(@(t, y) odefun(t, y, parr, destroyed_defenders), T:-delta:0, initr); yr = flip(yr)';
        for j = k:k+length(t0:delta:te)-2
            if j == 1, x(:,j) = xinit; end
            if j+1 <= length(time)
                c1 = find(destroyed_defenders);
                if ~isequal(c, c1)
                    parr = problemData_MD_rescue(destroyed_defenders);
                    initr = initializeHessians(parr);
                    [~, yr] = ode45(@(t, y) odefun(t, y, parr, destroyed_defenders), T:-delta:0, initr); yr = flip(yr)';
                end
                Aclr = calculateMatrices(yr, j, parr, destroyed_defenders);
                if size(Aclr,1) ~= 2*agents
                    rows_to_add = [2*c1+1,2*(c1+1)]; rows_to_add = sort(rows_to_add);
                    cols_to_add = rows_to_add; Aclr = expand_matrix_with_zeros(Aclr,rows_to_add,cols_to_add);
                end
                x(:,j+1) = expm(Aclr * (time(j+1) - time(j))) * x(:,j);
                [Xar, Xdr, Xtr] = updatePositions_2D(x, j+1, defender, destroyed_defenders);
                updatePlots_2D(Xa_path, Xd_paths, Xt_path, Xar, Xdr, Xtr, defender, destroyed_defenders);
                [capture, capture_position, capture_type, destroyed_defenders, evade_position] = ...
                    checkCaptureforrescue(Xar, Xdr, Xtr, parr, destroyed_defenders, time, j, evade_position);
                drawnow; pause(0.06);
            end
            if ~capture, break; end
            c = c1;
        end
        k = j + 1;
    end
    c = [];
    t0 = te;
    te = te + par.deltaRH;
    par = problemData_RH(x(1:2,end),x(end-1:end,end));
end

% Print switch log and outcome, as your original code
disp('==== Mode Switching Log: ====')
disp(mdata')
displayOutcome_2D(capture, capture_type, capture_position, pari, evade_position);

%% ---- Smooth Reference Generation for Feedback Linearization ----
finalIndex = min(k, maxSteps);
time_raw = time(1:finalIndex);
x_raw = x(:, 1:finalIndex);

t_interp = time_raw(1):delta_interp:time_raw(end);
x_interp = zeros(numState, length(t_interp));
for i = 1:numState
    x_interp(i,:) = interp1(time_raw, x_raw(i,:), t_interp, 'pchip');
end

total_agents = agents;
Ux = zeros(total_agents, length(t_interp)-1);
Uy = zeros(total_agents, length(t_interp)-1);
for j = 1:length(t_interp)-1
    dx = (x_interp(:,j+1) - x_interp(:,j)) / delta;
    for i = 1:total_agents
        Ux(i,j) = dx(2*i - 1);
        Uy(i,j) = dx(2*i);
    end
end

attackerTraj = x_interp(1:2,:);
targetTraj   = x_interp(3:4,:);
if defender > 0
    defenderTraj = x_interp(5:end,:);
else
    defenderTraj = [];
end

simTimeVec = t_interp;
capture_info.capture = capture;
capture_info.capture_position = capture_position;
capture_info.capture_type = capture_type;
capture_info.pari= pari;
capture_info.evade_position= evade_position;

end
