function displayOutcome_3D(capture, capture_type, capture_position, par,evade_position)
    if capture
        disp("No outcome");
    else
        if strcmp(capture_type, 'attacker')
            % plotCircle(capture_position(1), capture_position(2), par.sigma(1), 'r--', 'r');
            r = par.sigma(1);
            [X, Y, Z] = sphere(50);
            X = X * r; Y = Y * r; Z = Z * r;
            surf(X + capture_position(1), Y + capture_position(2), Z + capture_position(3), 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'r');
            plot3(capture_position(1), capture_position(2), capture_position(3), 'ro', 'MarkerFaceColor', 'r');
        else
            if strcmp(capture_type, 'defender2')
                r = par.sigma(3);
            else
                r=par.sigma(2);
            end
            [X, Y, Z] = sphere(50);
            X = X * r; Y = Y * r; Z = Z * r;
            surf(X + capture_position(1), Y + capture_position(2), Z + capture_position(3), 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'b');
            plot3(capture_position(1), capture_position(2), capture_position(3), 'bo', 'MarkerFaceColor', 'b');
        end
    end
    for i=1:(size(evade_position,1))
        [X, Y, Z]  = sphere(50);r = par.sigma(1);
        X = X * r; Y = Y * r; Z = Z * r;
        surf(X + evade_position(i,1), Y + evade_position(i,2), Z + evade_position(i,3), 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'g');
        plot3(evade_position(i,1), evade_position(i,2), evade_position(i,3));
    end
    % shading interp;
    light;
end