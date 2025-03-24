function displayOutcome_2D_MA(capture, capture_type, attacker, capture_positions, intercept_position, par)
    if capture
        disp("No outcome");
    else
        if startsWith(capture_type, 'attacker')
            plotCircle(capture_positions(1), capture_positions(2), par.sigma(1), 'r--', 'r');
        elseif strcmp(capture_type, 'all_attackers_destroyed')
            disp('All attackers have been intercepted by the defenders');
            % Loop through all interception positions and plot circle
        else
            if strcmp(capture_type, 'defender3')
                plotCircle(capture_positions(1), capture_positions(2), par.sigma(3), 'b--', 'b');
            else
                plotCircle(capture_positions(1), capture_positions(2), par.sigma(2), 'b--', 'b');
            end
        end
        legend off;
    end
    for i = 1:(size(intercept_position, 1)/2)
                plotCircle(intercept_position(2*i-1, 1), intercept_position(2*i, 1), par.sigma(2), 'g--', 'g'); % Adjust circle parameters if needed
    end
end

