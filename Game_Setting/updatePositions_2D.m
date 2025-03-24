function [Xa, Xd, Xt] = updatePositions_2D(x, k, defender, destroyed_defenders)
    Xa = x(1:2,k)';
    for i = 1:defender
        if ~destroyed_defenders(i)
            Xd{i} = x(2*i+1:2*(i+1),k)';
        else
            Xd{i} = [NaN, NaN]; % Placeholder for destroyed defenders
        end
    end
    Xt = x(2*(defender+1)+1:2*(defender+2),k)';
end