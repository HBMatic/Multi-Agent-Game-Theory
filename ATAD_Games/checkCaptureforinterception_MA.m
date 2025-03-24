function [capture, capture_position, intercept_position, capture_type, destroyed_defenders, destroyed_attackers] = checkCaptureforinterception_MA(Xa, Xd, Xt, par, defender, attacker, destroyed_defenders, destroyed_attackers, t, k, intercept_position)
    capture = 1;
    capture_position = [];
    capture_type = '';

    % Loop over each attacker
    for j = 1:attacker
        if ~destroyed_attackers(j)
            % Check if the attacker has reached the target
            if (norm(Xa{j} - Xt) <= par.sigma(1))
                defender_distance = arrayfun(@(i) norm(Xd{i} - Xa{j}), find(~destroyed_defenders));
                if all(defender_distance > par.sigma(2))
                    fprintf('Target is captured by Attacker %d at %d sec\n', j, double(t(k,1)));
                    capture = 0;
                    capture_position = Xa{j};
                    capture_type = sprintf('attacker%d', j);
                    return; % Terminate function as target is captured
                end
            else
                for i = find(~destroyed_defenders)
                    if i == 3
                        if norm(Xd{i} - Xa{j}) <= par.sigma(3)
                            fprintf('Defender %d intercepts Attacker %d at %d sec\n', i, j, double(t(k,1)));
                            destroyed_attackers(j) = true;
                            intercept_position = [intercept_position; Xd{i}];
                            break;
                        elseif norm(Xd{i} - Xa{j}) <= par.sigma(1)
                            fprintf('Defender %d is destroyed by Attacker %d at %d sec\n', i, j, double(t(k,1)));
                            destroyed_defenders(i) = true;
                        end
                    else
                        if norm(Xd{i} - Xa{j}) <= par.sigma(2)
                            fprintf('Defender %d intercepts Attacker %d at %d sec\n', i, j, double(t(k,1)));
                            destroyed_attackers(j) = true;
                            intercept_position = [intercept_position; Xd{i}];
                            break;
                        elseif norm(Xd{i} - Xa{j}) <= par.sigma(1)
                            fprintf('Defender %d is destroyed by Attacker %d at %d sec\n', i, j, double(t(k,1)));
                            destroyed_defenders(i) = true;
                        end
                    end
                end
            end
        end
    end

    % Check if all attackers are destroyed
    if all(destroyed_attackers)
        capture = 0;
        capture_type = 'all_attackers_destroyed';
    end
end