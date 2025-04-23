%% simulateInterception_withInterpolation.m
% Simulates interception, then smooths trajectories by interpolation to reduce Ux, Uy

%% 1) Initialize Initial Positions
attacker_init = [1; 1];
target_init   = [1; -1];
defenders_init = [-1, -1; -1, 1];

%% 2) Run original simulation
[attackerTraj, targetTraj, defenderTraj, time, capture_info, Ux_raw, Uy_raw] = ...
    simulateInterception(attacker_init, target_init, defenders_init);

%% 3) Combine all trajectories into a single x matrix
x_raw = [attackerTraj; targetTraj; defenderTraj];
total_agents = size(x_raw,1)/2;

%% 4) Interpolate Trajectories
interp_factor = 12;  % 4x more points → smoother velocities
t_interp = linspace(time(1), time(end), interp_factor * length(time));
x_interp = zeros(size(x_raw,1), length(t_interp));

for i = 1:size(x_raw,1)
    x_interp(i,:) = interp1(time, x_raw(i,:), t_interp, 'pchip');
end

%% 5) Recompute Ux and Uy from interpolated trajectory
Ux = zeros(total_agents, length(t_interp)-1);
Uy = zeros(total_agents, length(t_interp)-1);

for k = 1:length(t_interp)-1
    dx = (x_interp(:,k+1) - x_interp(:,k)) / 0.05;
    for i = 1:total_agents
        Ux(i,k) = dx(2*i - 1);
        Uy(i,k) = dx(2*i);
    end
end

%% 6) Extract smoothed trajectories
attackerTraj = x_interp(1:2,:);
targetTraj   = x_interp(3:4,:);
defenderTraj = x_interp(5:end,:);
time         = t_interp;

%% 7) (Optional) Plot comparison
figure;
subplot(2,1,1); plot(Ux'); title('Interpolated Ux'); xlabel('Step'); ylabel('Ux');
subplot(2,1,2); plot(Uy'); title('Interpolated Uy'); xlabel('Step'); ylabel('Uy');

%% 8) Your smoothed variables ready:
% attackerTraj, targetTraj, defenderTraj: all interpolated
% Ux, Uy: velocities with reduced magnitude
% time: new time vector
