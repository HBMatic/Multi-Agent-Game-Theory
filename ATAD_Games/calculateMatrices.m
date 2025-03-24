function Acl = calculateMatrices(y, k,par, destroyed_defenders)
    L = (par.n*(par.n+1)/2);
    defender = par.defender;
    Pa = symgen(y(1:L, k));
    for i = 1:defender
        Pd{i} = symgen(y(L*i+1:L*(i+1), k));
    end
    Pt = symgen(y(L*(defender+1)+1:L*(defender+2), k));
    Acl=[];
    Acl = -par.Sa*Pa;
    for i = 1:defender
        Acl = Acl - par.Sd{i} * Pd{i};
    end
    Acl = Acl - par.St * Pt;
end