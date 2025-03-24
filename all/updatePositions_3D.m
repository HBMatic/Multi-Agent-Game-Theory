function [Xa, Xd, Xt] = updatePositions_3D(x, k, defender, destroyed_defenders)
    Xa = x(1:3,k)';
    for i = 1:defender
        if ~destroyed_defenders(i)
            Xd{i} = x(3*i+1:3*(i+1),k)';
        else
            Xd{i} = [NaN, NaN, NaN]; % Placeholder for destroyed defenders
        end
    end
    Xt = x(3*(defender+1)+1:3*(defender+2),k)';
end