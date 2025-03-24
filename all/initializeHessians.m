function init = initializeHessians(par)
    % L=par.n*(par.n+1)/2;
    init = vectgen(par.Qa1); % Initialize with the attacker

    for i = 1:par.defender
        % if ~destroyed_defenders(i)
            init = [init; vectgen(par.(['Qd', num2str(i)]))];
        % else
        %     % Append a zero vector of the same size as vectgen would have returned
        %     zero_vector = zeros(L,1);
        %     init = [init; zero_vector];
        % end
    end
    
    init = [init; vectgen(par.Qt1)]; % Append the final element
end
