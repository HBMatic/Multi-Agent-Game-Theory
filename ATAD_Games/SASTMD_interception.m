clc; close all; clear all;

T = 5;
delta = 0.05;
time=[0:delta:T];
evade_position=[];
% defender = input('Enter number of Defender: ');
defender = 5;
agents = defender+2;
destroyed_defenders = false(1,defender);

par = problemData_MD_intercept(destroyed_defenders);
init = initializeHessians(par);
[t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init);
y = flip(y); y = y';t = flip(t);

xinit = [0.2276;0.1619;1.5924;2.3375;2.8020;0.3897;1.7065;1.4082];
xinit = 3*rand(2*(defender+2), 1);
%xinit = [2.1281;2.2641;0.8281;2.0391;.9653;0.4878;0.3570;1.4951;2.8792;1.0212;1.7558;0.6714];
% xinit = [0.5749;1.5894;1.8123;1.5455;0.0516;1.3975;0.2481;2.5163;0.9674,;2.6360]; %CS-1
% xinit = [-2;2;-1;1;1;1;1;-1;-0.6;1]; %  CS-2
% xinit = [-2;2;0;0;1;1.5;-1;0;0;0]; %CS-3
% xinit = [-0.5327;-0.0979;2.9085;1.1222;2.0470;0.7731;0.8375;1.6901;1.5558;-0.1203]; %CS-4
% xinit = [1.0767;1.9923;0.1564;0.8854;0.2133;1.9238;0.0093;1.5498;1.6346;1.7374;0.1689;0.7996]; % 4 defender
% xinit =[0.1576;0.9706;0.9572;0.4854;0.8003;0.1419;0.4218;0.9157;0.7922;0.9595;0.6557;0.0357;0.8491;0.9340];% five defender
% xinit = [1.0944;0.2772;0.2986;0.5150;1.6814;0.5086;1.6286;0.4870;1.8585;0.7000;0.3932;0.5022;1.2321;0.9466]; % five defender
% xinit = [1.9990;1.6174;2.0943;1.9996;0.5344;0.3840;2.9972;0.5134;0.0978;
%     1.6836;2.6456;2.0075;0.5713;1.1067]; % 5 defenders
% xinit = [0.2187;0.2656;2.3951;2.8290;2.0511;0.3962;2.1682;0.3311;0.3525;1.9222;0.9864;
%     1.9614;2.2474;1.7496];
% xinit = [ 2.7189; 2.6390;2.4533;0.7822;1.7831;0.0675];
% xinit = [0.7644;0.6721;2.0035;2.5332;1.0334;2.3416;2.0260;0.0201];
% xinit = [2.8020;0.3897;1.7065;1.4082;0.0357;1.0114;0.4865;2.3829;0.9336;1.5856;0.4969;1.8059];
capture = 1;

figure;
hold on;
xlabel('X Position');
ylabel('Y Position');
title('Trajectories of Attacker, Defenders, and Target : Interception Mode');
axis equal;
[Xa_path, Xd_paths, Xt_path] = initializePlots_2D(xinit, defender);
k=1;c=[];
while capture 
    if k == 1
        x(:,1) = xinit;
    end
    c1=find(destroyed_defenders);
    if ~isequal(c,c1)
        par = problemData_MD_intercept(destroyed_defenders);
        init = initializeHessians(par);
        [t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:time(k), init);
        y = flip(y); y = y';t = flip(t);
    
    end
    Acl = calculateMatrices(y, k, par, destroyed_defenders);    

    if size(Acl)~=size(zeros(2*agents))
        rows_to_add = [2*c1+1,2*(c1+1)];
        rows_to_add = sort(rows_to_add);
        cols_to_add = [2*c1+1,2*(c1+1)];
        cols_to_add = sort(cols_to_add);
        Acl = expand_matrix_with_zeros(Acl,rows_to_add,cols_to_add);
    end
    x(:,k+1) = expm(Acl*(time(k+1)-time(k)))*x(:,k);
    [Xa, Xd, Xt] = updatePositions_2D(x, k+1, defender, destroyed_defenders);
    updatePlots_2D(Xa_path, Xd_paths, Xt_path, Xa, Xd, Xt, defender, destroyed_defenders);
    [capture, capture_position, capture_type, destroyed_defenders,evade_position] = checkCaptureforinterception(Xa, Xd, Xt, par, destroyed_defenders, time, k,evade_position);
  
    if ~capture
        break;
    end
    k=k+1;
    c=c1;
    drawnow;
    pause(0.06);
end

displayOutcome_2D(capture, capture_type, capture_position, par,evade_position);