function [Pa, Pd, Pt, Acl] = calculateMatrices_MA(y, k, defender, attacker, par, destroyed_defenders, destroyed_attackers)
    L = (par.n*(par.n+1)/2);
    
    % Initialize Pa, Pd, Pt, Acl
    Pa = cell(1, attacker);
    Pd = cell(1, defender);
    
    % Generate Pa matrices for each attacker
    for j = 1:attacker
        Pa{j} = symgen(y((j-1)*L+1:j*L, k));
    end
    
    % Generate Pd matrices for each defender
    for i = 1:defender
        Pd{i} = symgen(y((attacker*L)+(i-1)*L+1:(attacker*L)+i*L, k));
    end
    
    % Generate Pt matrix
    Pt = symgen(y((attacker+defender)*L+1:(attacker+defender+1)*L, k));
    
    % Calculate Acl
    Acl = zeros(size(Pa{1})); % Initialize Acl with appropriate size
    for j = 1:attacker
        if ~destroyed_attackers(j)
            Acl = Acl - par.Sa{j} * Pa{j};
        end
    end
    for i = 1:defender
        if ~destroyed_defenders(i)
            Acl = Acl - par.Sd{i} * Pd{i};
        end
    end
    Acl = Acl - par.St * Pt;
end
