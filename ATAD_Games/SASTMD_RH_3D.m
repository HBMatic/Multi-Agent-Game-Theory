clc;close all;clear all;

mdata = [];
T = 5;
delta = 0.01;
time = [0:delta:T];
% defender = input('Enter No of Defender: ');
defender = 4;
agents = defender+2;
destroyed_defenders = false(1,defender);
evade_position = [];
xinit = 3 * rand(3 * agents, 1);
% xinit = [0.2598;1.1376;0.9388;0.0238;0.6742;0.3244;1.5886;0.6224;
%     1.0571;0.3313;1.2040;0.5259;1.3082;1.3784;1.4963];
% xinit = [0.9011;0.1676;0.4580;1.8267;0.3048;1.6516;1.0767;1.9923;0.1564;
%     0.8854;0.2133;1.9238;0.0093;1.5498;1.6346];
% xinit = [1.7374;0.1689;0.7996;0.5197;1.6001;0.8628;1.8213;0.3637;0.5276;
%     0.2911;0.2721;1.7386;1.1594;1.0997;0.2899];
% xinit = [0.4696;0.7063;1.6424;0.0308;0.0860;0.3380;1.2982;1.4634;1.2955;
%     0.9018;1.0940;0.5926;1.4894;0.3779;1.3736];
% xinit = [1.0617;1.3089;0.8152;1.6400;1.4367;1.9373;1.0627;0.6503;0.2113;
%     1.2219;1.5576;0.8469;0.1816;0.5329;0.3073];
% xinit = [1.9768;1.0800;1.4138;1.9990;0.5757;0.8290;0.9297;1.5279;1.6364;
%     0.2004;0.3562;0.7193;0.1134;1.0438;0.6717];
% xinit = [ 0.3718;1.4711;2.5590;2.6218;0.8109;0.6254;1.6949;1.9209;
%     1.2511;0.6179;2.8438;0.2462;0.3171;0.4261;0.4994];
% xinit = [2.6809;0.1644;0.9110;0.1386;0.5864;2.1605;2.1653;2.6334;
%     1.7473;0.2121;2.7682;2.4011;0.8578;1.6310;2.9543];
% xinit = [0.1486;1.4687;0.5775;0.3693;0.6165;0.4395;0.5672;0.1280;
%     1.9056;0.8456;1.6158;2.0855;1.4973;1.6074;1.3355];

xinit = [0.6181;0.2600;2.3158;0.6170;1.1648;1.6553;0.6869;1.9258;
    1.4534;0.4555;2.3458;0.3018; 0.8822;0.7121;1.5926;0.2745;1.2159;0.3145];
% % xinit = [2.7414;2.1201;1.6734;0.9403;0.4986;1.8675;2.9638;0.5113;0.7734;1.1904;0.2220;2.0523;
% %     1.2072;2.9485;1.2066;1.8620;0.4631;1.1440];
% xinit = [0.3746;0.0733;0.8706;0.9526;1.9611;2.8708;2.8072;1.3737;
%     0.7214;2.2917;2.2780;2.2219;2.2311;0.3178;2.0447;1.3898;0.6365;0.956];
Xa = xinit(1:3, 1);
Xd = cell(1,defender);
for i = 1:defender
    Xd{i} = xinit(3 * i + 1:3 * (i + 1), 1);
end
Xt = xinit(3 * (defender + 1) + 1:3 * (defender + 2), 1);

par = problemData_RH(Xa, Xt);
t0 = 0;
te = t0 + par.deltaRH;

%%%%%%%%%% Interception Mode %%%%%%%%%%%
pari = problemData_MD_intercept_3D(destroyed_defenders);
initi = initializeHessians(pari);
%%%%%%%%%%% Rescue Mode %%%%%%%%%%%%%%%
parr = problemData_MD_rescue_3D(destroyed_defenders);
initr = initializeHessians(parr);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
capture = 1;
figure;
hold on;
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
txt = {['Trajectories of Attacker, Defenders, and Target (\mu =',num2str(parr.meu),',\lambda =',num2str(pari.lamda),') '],': RH Mode (\sigma_a > \sigma_d_i)'};
title(txt);
view(3); grid on;
axis equal;
[Xa_path, Xd_paths, Xt_path] = initializePlots_3D(xinit, defender);

k = 1;
c = [];
while capture && te <=T
    if par.psi <= 0
        mode = 'intercept';
    else
        mode = 'rescue';
    end
    mdata = [mdata; string(mode)];


    if strcmp(mode, 'intercept')
        for j = k:k+length(t0:delta:te)-2
            if j == 1
                x(:, j) = xinit;
            elseif j == k
                % xi(:, j) = xini;
            end
            if j + 1 <= length(time)
                c1 = find(destroyed_defenders);
                if ~isequal(c,c1)
                    pari  = problemData_MD_intercept_3D(destroyed_defenders);
                    initi = initializeHessians(pari);
                    [t, yi] = ode45(@(t,y) odefun(t,y,pari,destroyed_defenders), T:-delta:0, initi);
                    yi = flip(yi)'; t = flip(t);
                end
                Acli = calculateMatrices(yi, j ,pari, destroyed_defenders);
                if size(Acli) ~= size(zeros(3*agents))
                    rows_to_add = [3*c1+1,3*c1+2,3*(c1+1)]; rows_to_add = sort(rows_to_add);
                    cols_to_add = [3*c1+1,3*c1+2,3*(c1+1)]; cols_to_add = sort(cols_to_add);
                    Acli = expand_matrix_with_zeros(Acli,rows_to_add,cols_to_add);
                end
               x(:, j + 1) = expm(Acli * (time(j + 1) - time(j))) * x(:, j);
               [Xai, Xdi, Xti] = updatePositions_3D(x, j+1, defender, destroyed_defenders);
               updatePlots_3D(Xa_path, Xd_paths, Xt_path, Xai, Xdi, Xti, defender, destroyed_defenders);
               [capture, capture_position, capture_type, destroyed_defenders,evade_position] = checkCaptureforinterception(Xai, Xdi, Xti, pari, destroyed_defenders, time, j,evade_position);
               drawnow;pause(0.06);
            end
            if ~capture
                break;
            end
            c = c1;
            if j<=length(time)
                [Xai,Xdi,Xti] = updatePositions_3D(x, j+1, defender,destroyed_defenders);
                xini = [Xai'; cell2mat(Xdi)';Xti'];
            end
        end        
        k = j + 1;

    else
        for j = k:k+length(t0:delta:te)-2
            if j == 1
                x(:, j) = xinit;
            else 
                % xr(:, j) = xini;
            end
            if j + 1 <= length(time)
                c1 = find(destroyed_defenders);
                if ~isequal(c,c1)
                    parr = problemData_MD_rescue_3D(destroyed_defenders);
                    initr = initializeHessians(parr);
                    [t, yr] = ode45(@(t,y) odefun(t,y,parr,destroyed_defenders), T:-delta:0,initr);
                    yr = flip(yr)'; t = flip(t);
                end
                Aclr = calculateMatrices(yr,j, parr, destroyed_defenders);
                if size(Aclr) ~= size(zeros(3*agents))
                    rows_to_add = [3*c1+1,3*c1+2,3*(c1+1)]; rows_to_add = sort(rows_to_add);
                    cols_to_add = [3*c1+1,3*c1+2,3*(c1+1)]; cols_to_add = sort(rows_to_add);
                    Aclr = expand_matrix_with_zeros(Aclr,rows_to_add,cols_to_add);
                end
                x(:, j + 1) = expm(Aclr * (time(j + 1) - time(j))) * x(:, j);
                [Xar, Xdr, Xtr] = updatePositions_3D(x, j+1, defender, destroyed_defenders);
                updatePlots_3D(Xa_path, Xd_paths, Xt_path, Xar, Xdr, Xtr, defender, destroyed_defenders);
                [capture, capture_position, capture_type, destroyed_defenders,evade_position] = checkCaptureforrescue(Xar, Xdr, Xtr, parr,destroyed_defenders, time, j,evade_position);               
                drawnow; pause(0.06);
            end
            if ~capture
                break;
            end
            c = c1;
            if j<= length(time)
                [Xar, Xdr, Xtr] = updatePositions_3D(x, j+1,defender,destroyed_defenders);
                xini = [Xar';cell2mat(Xdr)';Xtr'];
            end
        end
        k = j + 1;
    end
    c = [];
    t0 = te;
    te = te + par.deltaRH;
    T=T+par.deltaRH;
    par = problemData_RH(x(1:3,end), x(end-2:end,end));
end
mdata
displayOutcome_3D(capture, capture_type, capture_position, pari, evade_position);
 % Compute norm(Xa - Xt) and par.kappa * par.sigma(1) for each time step
norm_diff = arrayfun(@(i) norm(x(1:3, i) - x(end-2:end, i)), 1:k);
kappa_sigma = par.kappa * par.sigma(1);
figure;
time = time(1:k);
plot(time, norm_diff, 'r', 'DisplayName', '||Xa - Xt||');
hold on;
plot(time, kappa_sigma * ones(1,k), 'b', 'DisplayName', '\kappa * \sigma_{a}');
xlabel('Time');
ylabel('Values'); txt = '||(Xa - Xt)|| and \kappa * \sigma_{a} vs Time';
title(txt);
legend show;
hold off;

% fig2plotly(gcf, 'offline', true, 'filename', '3D_Plot_Example.html');


