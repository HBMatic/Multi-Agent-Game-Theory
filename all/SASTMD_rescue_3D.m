clc;close all;clear all;

T=5;
delta = 0.05;
time = [0:delta:T];
defender = 4;
agents = defender+2;
destroyed_defenders = false(1,defender);
evade_position = [];
par= problemData_MD_rescue_3D(destroyed_defenders);
init = initializeHessians(par);

[t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders), T:-delta:0, init); % backward in time 
y = flip(y)'; t= flip(t);

xinit = 2*rand(3*(agents),1);
% xinit = [0.7015;1.8780;1.7519;1.1003;1.2450;1.1741;0.4155;0.6025;
%     0.9418;0.4610;1.6886;0.3895;0.4518;0.3414;0.4553;0.8714;0.6222;1.8468];
xinit = [1.9759;0.3409;0.5156;0.7936;0.1480;1.3682;0.8048;1.9657;
    0.8044;1.2413;0.3087;0.7627;0.3223;1.5162;1.7422;0.7016;1.3711;0.5883];
capture = 1;
figure;
hold on;
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
title('Trajectories of Attacker, Defenders, and Target: Rescue Mode');
axis equal;
grid on;
view(3);
[Xa_path, Xd_paths, Xt_path] = initializePlots_3D(xinit,defender);
k=1;c=[];
while capture
    if k==1
        x(:,1) = xinit;
    end
    c1 = find(destroyed_defenders);
    if ~isequal(c,c1)
        par = problemData_MD_rescue_3D(destroyed_defenders);
        init = initializeHessians(par); 
        [t, y] = ode45(@(t,y) odefun(t,y,par,destroyed_defenders),T:-delta:time(k),init);
        y = flip(y)'; t = flip(t);
    end
    Acl = calculateMatrices(y,k,par,destroyed_defenders);
    if size(Acl)~=size(zeros(3*agents))
         rows_to_add = [3*c1+1,3*c1+2,3*(c1+1)];
        rows_to_add = sort(rows_to_add);
        cols_to_add = [3*c1+1,3*c1+2,3*(c1+1)];
        cols_to_add = sort(cols_to_add);
        Acl = expand_matrix_with_zeros(Acl,rows_to_add,cols_to_add);
    end
    x(:,k+1) = expm(Acl*(time(k+1)-time(k)))*x(:,k);
    [Xa, Xd, Xt] = updatePositions_3D(x, k+1, defender,destroyed_defenders);
    updatePlots_3D(Xa_path,Xd_paths,Xt_path,Xa,Xd,Xt,defender,destroyed_defenders);
    [capture, capture_position, capture_type, destroyed_defenders,evade_position] = checkCaptureforrescue(Xa,Xd,Xt,par,destroyed_defenders,time,k,evade_position);

    if ~capture
        break
    end
    k=k+1;
    c=c1;
    drawnow;
    pause(0.06);
end
displayOutcome_3D(capture,capture_type,capture_position,par,evade_position);








