function par = problemData_MATD_intercept(defender, attacker, target, xinit)
    % Parameters
    par.sigma(1) = 0.1; % Attacker's capture radius
    par.sigma(2) = 0.1; % Small defender radius
    par.sigma(3) = 0.1; % Big defender capture radius
    par.meu = 1;
    agents = defender + target + attacker; % Total number of agents
    
    syms Xt_1 Xt_2 real;
    Xt = [Xt_1; Xt_2];
    
    % Initialize state variables
    par.n = 2 * agents;
    X = sym([zeros(2 * (defender + attacker), 1); Xt]);
    
    I = eye(agents);
    par.A = zeros(par.n, par.n);
    par.Bt = kron(I(:, defender + attacker + 1), eye(2, 2));
    par.Rt = 1.25 * eye(2, 2);
    par.St = par.Bt * (par.Rt \ par.Bt');
    par.Qt = eye(2, 2);
    
    for i = 1:attacker
        par.Ba{i} = kron(I(:, i), eye(2, 2));
        par.Ra{i} = 0.75 * eye(2, 2);
        par.Sa{i} = par.Ba{i} * (par.Ra{i} \ par.Ba{i}');
        X(2 * i - 1:2 * i) = sym(['Xa' num2str(i) '_1'; 'Xa' num2str(i) '_2']);
    end
    
    for i = 1:defender
        par.Bd{i} = kron(I(:, attacker + i), eye(2, 2));
        par.Rd{i} = eye(2, 2);
        par.Sd{i} = par.Bd{i} * (par.Rd{i} \ par.Bd{i}');
        X(2 * (i + attacker) - 1:2 * (i + attacker)) = sym(['Xd' num2str(i) '_1'; 'Xd' num2str(i) '_2']);
    end
    
    pairs = matching(xinit, defender, attacker); % Find pairs based on minimum distance
    
    % Objective function
    obj = arrayfun(@(x) {0}, zeros(agents, 1));
    
    for i = 1:attacker
        Xa_i = X(2 * i - 1:2 * i);
        Xdmin_i = X(2 * (attacker + pairs(i, 2)) - 1:2 * (attacker + pairs(i, 2)));
        obj{i} = (Xa_i - Xt)' * eye(2, 2) * (Xa_i - Xt) - (Xa_i - Xdmin_i)' * eye(2, 2) * (Xa_i - Xdmin_i);
        obj{defender + attacker + 1} = obj{defender + attacker + 1} - (Xt - Xa_i)' * eye(2, 2) * (Xt - Xa_i);
    end

    for i = 1:defender
        Xd_i = X(2 * (i + attacker) - 1:2 * (i + attacker));
        for j = 1:attacker
            Xa_j = X(2 * j - 1:2 * j);
            obj{i + attacker} = obj{i + attacker} + (Xd_i - Xa_j)' * eye(2, 2) * (Xd_i - Xa_j); % Each defender vs all attacker
        end
        obj{defender + attacker + 1} = obj{defender + attacker + 1} + par.meu * (Xt - Xd_i)' * eye(2, 2) * (Xt - Xd_i);
    end
    % obj{defender + attacker + 1} = -obj{defender + attacker + 1};
    
    % Hessians
    for i = 1:attacker
        par.(['Qa', num2str(i)]) = 0.5 * double(hessian(obj{i}, X));
    end
    
    for i = 1:defender
        par.(['Qd', num2str(i)]) = 0.5 * double(hessian(obj{i + attacker}, X));
    end
    
    par.Qt1 = 0.5 * double(hessian(obj{defender + attacker + 1}, X));
end
