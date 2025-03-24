function pairs = matching(xinit, defender, attacker)
    % Extract attacker coordinates
    Xa = cell(1, attacker);
    for i = 1:attacker
        Xa{i} = xinit(2*i-1:2*i);
    end

    % Extract defender coordinates
    Xd = cell(1, defender);
    for i = 1:defender
        Xd{i} = xinit(2*(attacker + i) - 1:2*(attacker + i));
    end

    % Calculate distance matrix
    distance = zeros(defender, attacker);
    for i = 1:attacker
        for j = 1:defender
            distance(j, i) = norm(Xa{i} - Xd{j});
        end
    end

    % Find the smallest distance for each attacker
    pairs = zeros(attacker, 2); % First column for attacker, second for defender
    for i = 1:attacker
        [~, minIndex] = min(distance(:, i));
        pairs(i, :) = [i, minIndex];
    end
end
