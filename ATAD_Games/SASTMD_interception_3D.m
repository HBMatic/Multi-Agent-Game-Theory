clc; clear all;close all;

% Parameters
T = 5; 
delta = 0.05;
time = [0:delta:T];
% defender = input('Enter number of Defender: ');
defender = 4; % Set number of defenders here
agents = defender+2;
destroyed_defenders = false(1,defender);
% Get problem data
evade_position = [];
par = problemData_MD_intercept_3D(destroyed_defenders);
init = initializeHessians(par);
% Solve ODE
[t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init); % backward in time 
y = flip(y); y = y'; t = flip(t);

% Initialize storage matrices
% L = (par.n*(par.n+1)/2);

% Initial state vector
xinit = 3*rand(3*(agents),1);
% xinit = [0.7448;0.3962;0.9794;0.6790;1.9033;1.8407;0.1054;1.4757;
%     0.5382;0.8457;1.0957;1.8855;0.8355;1.9661; 0.6029];
% xinit = [0.8604;0.3696;1.8098;1.9595;0.8777;0.2222;0.5161;0.8174;
%     1.1898;0.5244;1.2057;1.4224;0.4435;0.2348;0.5934];
% 
% xinit =[0.0505;1.6844;1.1181;1.7082;0.6958;0.8921;0.1085;0.3542;1.3256
%     ;0.6617;1.7970;0.2363;1.9768;1.0800;1.4138;1.9990;
%     0.5757;0.8290;0.9297;1.5279;1.6364];
% xinit = [0.0689;0.8775;0.7631;1.5310;1.5904;0.3737;0.9795;0.8912;1.2926;1.4187;
%     1.5094;0.5521;1.3594;1.3102;0.3252;0.2380;0.9967;1.9195;0.6808;1.1705;0.4476];
% xinit = [1.6617;1.1705;1.0994;1.8344;0.5717;1.5144;1.5075;0.7609;1.1356;0.1517;0.1079;
%     1.0616;1.5583;1.8680;0.2598;1.1376;0.9388;0.0238;0.6742;0.3244;1.5886];
% xinit = [1.6294;1.8116;0.2540;1.8268;1.2647;0.1951;0.5570;1.0938;
%     1.9150;1.9298;0.3152;1.9412;1.9143;0.9708;1.6006;0.2838;0.8435;1.8315];% 4defenders
capture = 1;

% Plot settings
figure;
hold on;
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
title('Trajectories of Attacker, Defenders, and Target: Interception Mode');
axis equal;
grid on;
view(3);
c=[];
[Xa_path, Xd_paths, Xt_path] = initializePlots_3D(xinit,defender);
k=1;
while capture
    if k==1
        x(:,1) = xinit;
    end
    c1 = find(destroyed_defenders);
    if ~isequal(c,c1)
        par= problemData_MD_intercept_3D(destroyed_defenders);
        init = initializeHessians(par);
        [t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init);
        y = flip(y)';t= flip(t);
    end
    Acl = calculateMatrices(y,k,par,destroyed_defenders);
    if size(Acl)~= size(zeros(3*agents))
        % keyboard;
        rows_to_add = [3*c1+1,3*c1+2,3*(c1+1)];
        rows_to_add = sort(rows_to_add);
        cols_to_add = [3*c1+1,3*c1+2,3*(c1+1)];
        cols_to_add = sort(cols_to_add);
        Acl = expand_matrix_with_zeros(Acl,rows_to_add,cols_to_add);
    end
    x(:,k+1) = expm(Acl*(time(k+1)-time(k)))*x(:,k);
    [Xa, Xd, Xt] = updatePositions_3D(x, k+1, defender,destroyed_defenders);
    updatePlots_3D(Xa_path,Xd_paths,Xt_path, Xa, Xd, Xt, defender, destroyed_defenders);
    [capture, capture_position, capture_type, destroyed_defenders,evade_position] = checkCaptureforinterception(Xa, Xd, Xt, par,destroyed_defenders, time, k,evade_position);

    if ~capture
        break
    end
    k=k+1;
    c=c1;
    drawnow;
    pause(0.06);
end
displayOutcome_3D(capture, capture_type,capture_position, par, evade_position);

