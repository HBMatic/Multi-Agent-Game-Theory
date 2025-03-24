function dydt = odefun_MA(t, y, par, num_defenders, num_attackers)
    % num_defenders: Number of defenders
    % num_attackers: Number of attackers

    L = (par.n * (par.n + 1) / 2);

    % Initialize Pa (Attackers)
    Pa = cell(1, num_attackers);
    for i = 1:num_attackers
        Pa{i} = symgen(y((i-1)*L+1:i*L));
    end

    % Initialize Pd (Defenders)
    Pd = cell(1, num_defenders);
    for i = 1:num_defenders
        start_idx = num_attackers * L + (i-1) * L + 1;
        end_idx = num_attackers * L + i * L;
        Pd{i} = symgen(y(start_idx:end_idx));
    end

    % Initialize Pt (Target)
    Pt_start_idx = num_attackers * L + num_defenders * L + 1;
    Pt_end_idx = num_attackers * L + num_defenders * L + L;
    Pt{1} = symgen(y(Pt_start_idx:Pt_end_idx));

    % Compute Acl (Closed-loop system matrix)
    Acl = par.A;
    for i = 1:num_attackers
        Acl = Acl - par.Sa{i} * Pa{i};
    end
    for i = 1:num_defenders
        Acl = Acl - par.Sd{i} * Pd{i};
    end
    Acl = Acl - par.St * Pt{1};

    % Compute derivatives for attackers
    dPa = cell(1, num_attackers);
    for i = 1:num_attackers
        dPa{i} = -transpose(Acl) * Pa{i} - Pa{i} * Acl - par.(['Qa', num2str(i)]) - Pa{i} * par.Sa{i} * Pa{i};
    end

    % Compute derivatives for defenders
    dPd = cell(1, num_defenders);
    for i = 1:num_defenders
        dPd{i} = -transpose(Acl) * Pd{i} - Pd{i} * Acl - par.(['Qd', num2str(i)]) - Pd{i} * par.Sd{i} * Pd{i};
    end

    % Compute derivative for the target
    dPt = -transpose(Acl) * Pt{1} - Pt{1} * Acl - par.Qt1 - Pt{1} * par.St * Pt{1};

    % Convert matrix derivatives to vectors
    dy_a = [];
    for i = 1:num_attackers
        dy_a = [dy_a; vectgen(dPa{i})];
    end

    dy_d = [];
    for i = 1:num_defenders
        dy_d = [dy_d; vectgen(dPd{i})];
    end

    dy_t = vectgen(dPt);

    % Combine all derivatives into a single vector
    dydt = [dy_a; dy_d; dy_t];
end
