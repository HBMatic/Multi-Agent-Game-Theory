function [Xa, Xd, Xt] = updatePositions_2D_MA(x, k, defender, attacker, destroyed_defenders, destroyed_attackers)
    Xa = cell(1, attacker);
    Xd = cell(1, defender);

    for i = 1:attacker
        if ~destroyed_attackers(i)
            Xa{i} = x(2 * (i - 1) + 1 : 2 * i, k);
        end
    end

    for i = 1:defender
        if ~destroyed_defenders(i)
            Xd{i} = x(2 * (attacker + i - 1) + 1 : 2 * (attacker + i), k);
        end
    end

    Xt = x(2 * (attacker + defender) + 1 : 2 * (attacker + defender + 1), k);
end
