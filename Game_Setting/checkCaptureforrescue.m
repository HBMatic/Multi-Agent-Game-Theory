function [capture, capture_position, capture_type, destroyed_defenders,evade_position] = checkCaptureforrescue(Xa, Xd, Xt, par,  destroyed_defenders, t, k,evade_position)
    capture = 1;
    capture_position = [];
    capture_type = '';
    defender = par.defender;

    if (norm(Xa - Xt) <= par.sigma(1))
        defender_distance = arrayfun(@(i) norm(Xd{i} - Xt), find(~destroyed_defenders));
        if all(defender_distance > par.sigma(2))
            fprintf('Target is captured by the Attacker at %d sec\n', double(t(1,k+1)));
            capture = 0;
            capture_position = Xa;
            capture_type = 'attacker';
        end
    else
        for i = find(~destroyed_defenders)
            if i == 2
                if norm(Xd{i} - Xt) <= par.sigma(3) && norm(Xd{i} - Xa) > par.sigma(1)
                    fprintf('Defender %d rescue the Target at %d sec\n', i, double(t(1,k+1)));
                    capture = 0;
                    capture_position = Xd{i};
                    capture_type = sprintf('defender%d', i);
                    break;
                elseif norm(Xd{i} - Xa) <= par.sigma(1)
                    fprintf('Defender %d is destroyed by the Attacker at %d sec\n', i, double(t(1,k+1)));
                    destroyed_defenders(i) = true;
                    evade_position = [evade_position;Xa];
                end
            else
                if norm(Xd{i} - Xt) <= par.sigma(2) && norm(Xd{i} - Xa) > par.sigma(1)
                    fprintf('Defender %d rescue the Target at %d sec\n', i, double(t(1,k+1)));
                    capture = 0;
                    capture_position = Xd{i};
                    capture_type = sprintf('defender%d', i);
                    break;
                elseif norm(Xd{i} - Xa) <= par.sigma(1)
                    fprintf('Defender %d is destroyed by the Attacker at %d sec\n', i, double(t(1,k+1)));
                    destroyed_defenders(i) = true;
                    evade_position = [evade_position;Xa];
                end
            end
        end
    end
end