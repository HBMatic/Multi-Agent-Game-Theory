clc;close all; clear all;

T = 5;
delta = 0.05;
time = [0:delta:T];
% defender = input('Enter number of Defender: ');
defender = 4;
agents = defender+2;
destroyed_defenders = false(1,defender);
evade_position = [];
par = problemData_MD_rescue(destroyed_defenders);
init = initializeHessians(par);
[t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init);
y = flip(y); y = y'; t = flip(t);

xinit = 3*rand(2*(defender+2), 1);
% xinit = [0.1280;1.9056;0.8456;1.6158;2.0855;1.4973;1.6074;1.3355;
%     0.3718;1.4711;2.5590;2.6218];
% xinit = [2.1281;2.2641;0.8281;2.0391;1.9653;0.4878;0.3570;1.4951;2.8792;1.0212];
% xinit = [0.4571;2.4775;1.6150;2.9884;0.2345;1.3280;0.3200;2.8857;0.0139;2.3247;2.4519;2.6061];
% % xinit = [-2;-2;0;0;0;1];
% xinit = [-2;2;0;0;1;1.5;-1;0;0;1]; %CS-1 Page-37
% % xinit = [-1;2;-1;0.5;-1;1;0;1;0;1];
% % xinit = [2.9059;1.5940;0.9754;0.3169;1.8329;2.3364;1.2704;
% %     0.2725;0.7994;0.4610;0.8430;1.3203;1.5814;1.3723]; % 5 Defender

capture = 1;
figure;
hold on;
xlabel('X Position');
ylabel('Y Position');
title('Trajectories of Attacker, Defenders, and Target : Rescue Mode');
axis equal;
[Xa_path, Xd_paths, Xt_path] = initializePlots_2D(xinit, defender);
k=1;c=[];
while capture
    if k == 1
        x(:,1) = xinit;
    end
    c1 = find(destroyed_defenders);
    if ~isequal(c,c1)
        par = problemData_MD_rescue(destroyed_defenders);
        init = initializeHessians(par);
        [t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:time(k),init);
        y= flip(y)'; t=flip(t);
    end
    Acl = calculateMatrices(y, k, par, destroyed_defenders);
    if size(Acl) ~=size(zeros(2*agents));
        rows_to_add = [2*c1+1,2*(c1+1)];
        rows_to_add = sort(rows_to_add);
        cols_to_add = [2*c1+1,2*(c1+1)];
        cols_to_add = sort(cols_to_add);
        Acl = expand_matrix_with_zeros(Acl,rows_to_add,cols_to_add);
    end
    x(:,k+1) = expm(Acl*(time(k+1)-time(k)))*x(:,k);
    [Xa, Xd, Xt] = updatePositions_2D(x, k+1, defender, destroyed_defenders);
    updatePlots_2D(Xa_path, Xd_paths, Xt_path, Xa, Xd, Xt, defender, destroyed_defenders);
    [capture, capture_position, capture_type, destroyed_defenders,evade_position] = checkCaptureforrescue(Xa, Xd, Xt, par, destroyed_defenders, time, k,evade_position);
    
    if ~capture
        break;
    end
    k=k+1;
    c=c1;
    drawnow;
    pause(0.06);
end

displayOutcome_2D(capture, capture_type, capture_position, par,evade_position);
