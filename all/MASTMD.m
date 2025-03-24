clc;close all;clear all;

T = 5;
delta = 0.05;
target = 1;
attacker = input('Enter number of Attackers: ');
% attacker = 1;
defender = input('Enter number of Defendes: ');
% defender = 1;
% defender = input('Enter number of Defender: ');
agents = attacker + defender + target;

xinit = 2 * rand(2 * agents, 1);
% xinit = [2.0362;2.2732;2.2294;1.1767;1.9664;0.5136;2.1181;0.0955]; %a=2,,d=1,t=1
% xinit = [1.9432;1.3528;1.6410;0.8890;2.2341;0.5669;2.0603;0.5505;1.1055;1.8769]; %a=2,d=2,t=1
% xinit =[2.3407;0.2434;2.7882;2.3271;1.4604;1.3076;1.3404;0.9190;1.5255;1.5323]; %a=2,d=2,t=1
% xinit = [0.6029;1.4022;1.3327;1.0783;1.3962;1.3331;0.3563;0.2560;1.9982;0.3422];
% xinit= [2.7189;2.6390;2.4533;0.7822;1.7831;0.0675];
xinit = [1.6904;1.4773;1.1720;0.4935;1.3328;0.1670;1.2519;1.3219;
    1.4595;1.7815;1.9646;1.5381;1.1629;1.8566;1.1602;0.0340;
    0.2417;1.7254;0.9686;1.6897;0.4188;1.1046];%5
% xinit = [1.9004;0.0689;0.8775;0.7631;1.5310;1.5904;0.3737;0.9795;0.8912;1.2926];% a=2 d=2
% xinit=[0.8036;0.1519;0.4798;0.2466;0.3678;0.4799;0.8345;0.0993;1.8054;1.8896;0.9817;0.9785;0.6754;1.8001];%a=3,d=3
% xinit = [-1;1;-1;-1;1;1;1;-1;0;0];
par = problemData_MATD_intercept(defender, attacker, target, xinit);
init = initializeHessians_MA(par, defender, attacker);

[t, y] = ode45(@(t, y) odefun_MA(t, y, par, defender, attacker), T:-delta:0, init);
y = flip(y); y = y'; t = flip(t);

capture = 1;

figure;
hold on;
xlabel('X Position');
ylabel('Y Position');
title('Trajectories of Attackers, Defenders, and Target : Interception Mode');
axis equal;

[Xa_paths, Xd_paths, Xt_path] = initializePlots_2D_MA(xinit, defender, attacker);
destroyed_attackers = false(1, attacker);
destroyed_defenders = false(1, defender);
intercept_position = [];

for k = 1:length(t) - 1
    if k == 1
        x(:, 1) = xinit;
    end

    [Pa, Pd, Pt, Acl] = calculateMatrices_MA(y, k, defender, attacker, par, destroyed_defenders, destroyed_attackers);
    x(:, k + 1) = expm(Acl * (t(k + 1) - t(k))) * x(:, k);

    [Xa, Xd, Xt] = updatePositions_2D_MA(x, k + 1, defender, attacker, destroyed_defenders, destroyed_attackers);

    updatePlots_2D_MA(Xa_paths, Xd_paths, Xt_path, Xa, Xd, Xt, defender, attacker, destroyed_defenders, destroyed_attackers);

    [capture, capture_position,intercept_position, capture_type, destroyed_defenders, destroyed_attackers] = checkCaptureforinterception_MA(Xa, Xd, Xt, par, defender, attacker, destroyed_defenders, destroyed_attackers, t, k,intercept_position);

    if ~capture
        break;
    end
    drawnow;
    pause(0.06);
end

displayOutcome_2D_MA(capture, capture_type,attacker, capture_position,intercept_position, par);
