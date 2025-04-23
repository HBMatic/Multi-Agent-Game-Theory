function [traj_new, time_new, Ux_new, Uy_new] = reparameterize_trajectory(traj, Ux, Uy, time, v_max)
    % traj: 2×N
    % Ux, Uy: 1×N
    % time: 1×N
    % v_max: scalar

    v_mags = sqrt(Ux.^2 + Uy.^2);  % instantaneous speeds
    scale_factors = max(1, v_mags / v_max);

    % Fix: use length N-1, to avoid trailing empty time step
    dt_original = diff(time);             % length N-1
    dt_scaled = dt_original .* scale_factors(1:end-1);  % match length

    % New time base (length N)
    time_new = [0, cumsum(dt_scaled)];

    % Also truncate trajectories to match
    traj = traj(:,1:length(time_new));
    Ux = Ux(1:length(time_new));
    Uy = Uy(1:length(time_new));

    traj_new = traj;
    Ux_new = Ux;
    Uy_new = Uy;
end
