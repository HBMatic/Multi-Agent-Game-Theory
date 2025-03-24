function displayOutcome_2D(capture, capture_type, capture_position, par,evade_position)
    if capture
        disp("No outcome");
    else
        if strcmp(capture_type, 'attacker')
            plotCircle(capture_position(1), capture_position(2), par.sigma(1), 'r--', 'r');
        else
           if strcmp(capture_type, 'defender2')
                plotCircle(capture_position(1), capture_position(2), par.sigma(3), 'b--', 'b');
           else
                plotCircle(capture_position(1), capture_position(2), par.sigma(2), 'b--', 'b');
           end
        end
    end
    % legend off;
    for i = 1:(size(evade_position, 1))
        plotCircle(evade_position(i,1), evade_position(i, 2), par.sigma(1), 'g--', 'g'); % Adjust circle parameters if needed
    end

end




