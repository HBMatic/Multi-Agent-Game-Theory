function dydt = odefun(t, y, par, destroyed_defenders)

    L = (par.n*(par.n+1)/2);
    defender = par.defender;
    Pa = symgen(y(1:L));
    for i = 1:defender
        Pd{i} = symgen(y(L*i+1:L*(i+1)));
    end
    
    Pt = symgen(y(L*(defender+1)+1:L*(defender+2)));
    
    Acl = par.A - par.Sa * Pa;
    for i = 1:defender
        Acl = Acl - par.Sd{i} * Pd{i};
    end
    Acl = Acl - par.St * Pt;
    
    dPa = -transpose(Acl) * Pa - Pa * Acl - par.Qa1 - Pa * par.Sa * Pa;
    
    for i = 1:defender
        % if ~destroyed_defenders(i)
            dPd{i} = -transpose(Acl) * Pd{i} - Pd{i} * Acl - par.(['Qd', num2str(i)]) - Pd{i} * par.Sd{i} * Pd{i};
        % else
            % dPd{i} = zeros(L,L);
        % end
    end
    dPt = -transpose(Acl) * Pt - Pt * Acl - par.Qt1 - Pt * par.St * Pt;
    
    dy_a = vectgen(dPa);
    dy_d = [];
    for i = 1:defender
        dy_d = [dy_d; vectgen(dPd{i})];
    end
    dy_t = vectgen(dPt);
    
    dydt = [dy_a; dy_d; dy_t];
end