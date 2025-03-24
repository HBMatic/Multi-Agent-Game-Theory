function par = problemData_MD_rescue_3D(destroyed_defenders)
    par.defender = length(find(~destroyed_defenders));
    defender = par.defender;
    par.meu = 0;
    % Parameters
    par.sigma(1) = 0.3; % Attacker's capture radius
    par.sigma(2) = 0.1; % For all small_defender radius
    par.sigma(3) = 0.2; % One big defender capture radius
    agents = defender + 2;  % Total number of agents (1 attacker + n defenders + 1 target)
    
    % Symbolic variables
    syms Xa1 Xa2 Xa3 Xt1 Xt2 Xt3 real;
    Xa = [Xa1; Xa2; Xa3];
    Xt = [Xt1; Xt2; Xt3];
    
    % Initialize state variables for defenders
    par.n = 3 * agents;
    X =[Xa; sym(zeros(3*defender,1)); Xt];

    I = eye(agents);
    par.A = zeros(par.n, par.n);
    par.Ba = kron(I(:, 1), eye(3, 3));
    par.Bt = kron(I(:, defender + 2), eye(3, 3));
    par.Ra = 0.75 * eye(3, 3);
    par.Rt = 1.25 * eye(3, 3);
    par.Sa = par.Ba * (par.Ra \ par.Ba');
    par.St = par.Bt * (par.Rt \ par.Bt');
    par.Qa = eye(3, 3);
    par.Qt = eye(3, 3);

    for i = 1:defender
        par.Bd{i} = kron(I(:, i+1), eye(3, 3));
        par.Rd{i} = eye(3, 3);
        par.Sd{i} = par.Bd{i} / par.Rd{i} * par.Bd{i}';
        X(3*i+1:3*(i+1)) = sym(['Xd' num2str(i) '_1'; 'Xd' num2str(i) '_2';'Xd' num2str(i) '_3']);
    end
    
    % Objective functions
    obj{1} = (Xa - Xt)' * par.Qa * (Xa - Xt);  % Attacker
    obj{defender + 2} = -(Xt - Xa)' * par.Qt * (Xt - Xa);  % Target vs attacker
    
    for i = 1:defender
        Xd_i = X(3*i+1:3*(i+1));
        obj{i + 1} = (Xd_i - Xt)' * eye(3,3) * (Xd_i - Xt);  % Each defender vs target
        obj{defender + 2} = obj{defender + 2} + par.meu * ((Xt - Xd_i)' * par.Qt * (Xt - Xd_i));  % Target vs each defender
    end
    
    % Hessians
    par.Qa1 = 0.5 * double(hessian(obj{1}, X));
    for i = 1:defender
        par.(['Qd', num2str(i)]) = 0.5 * double(hessian(obj{i + 1}, X));
    end
    par.Qt1 = 0.5 * double(hessian(obj{defender + 2}, X));
end
