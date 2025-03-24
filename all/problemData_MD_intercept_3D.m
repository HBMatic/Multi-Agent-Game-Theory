function par = problemData_MD_intercept_3D(destroyed_defenders)
    par.defender = length(find(~destroyed_defenders));
    defender = par.defender;
    agents = par.defender+2;
    par.lamda=1;
    % Define capture radii
    par.sigma(1) = 0.3; % Attacker's capture radius
    par.sigma(2) = 0.1;% For all small_defender radius
    par.sigma(3) = 0.2; % One big defender capture radius
    
    % Symbolic variables for attacker and target positions
    syms Xa1 Xa2 Xa3 Xt1 Xt2 Xt3 real;
    
    % Initialize attacker and target vectors
    Xa = [Xa1; Xa2; Xa3];
    Xt = [Xt1; Xt2; Xt3];
    
    % Initialize general structures
    par.n = 3 * agents; % 3*(number of objects: attacker, defenders, target)
    X = [Xa; sym(zeros(3 * defender, 1)); Xt];
    
    % Initialize matrices
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
    
    % Initialize defender matrices
    for i = 1:defender
        par.Bd{i} = kron(I(:, i + 1), eye(3, 3));
        par.Rd{i} = eye(3, 3);
        par.Sd{i} = par.Bd{i} * (par.Rd{i} \ par.Bd{i}');
        X(3 * i + 1:3 * (i + 1)) = sym(['Xd' num2str(i) '_1'; 'Xd' num2str(i) '_2'; 'Xd' num2str(i) '_3']);
    end
    
    % Calculate objective functions and Hessians
    obj{1} = (Xa - Xt)' * par.Qa * (Xa - Xt);
    obj{defender+2} = -(Xt - Xa)' * par.Qt * (Xt - Xa);
    for i = 1:defender
        Xd_i = X(3 * i + 1:3 * (i + 1));
        obj{1} = obj{1} - par.lamda*(Xa - Xd_i)' * par.Qa * (Xa - Xd_i);
        obj{i+1} = (Xd_i - Xa)' * eye(3, 3) * (Xd_i - Xa);
    end
    
    % Calculate Hessians
    par.Qa1 = 0.5 * double(hessian(obj{1}, X));
    par.Qt1 = 0.5 * double(hessian(obj{defender+2}, X));
    for i = 1:defender
        par.(['Qd' num2str(i)]) = 0.5 * double(hessian(obj{i+1}, X));
    end
end
