function plot_3d(xinit, x, t, defender, capture, capture_type, capture_position, par)
    figure;
    hold on;
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    title('Trajectories of Attacker, Defenders, and Target: Interception Mode');
    axis equal;
    grid on;
    view(3);

    % Initialize animated lines
    Xa_path = animatedline('Color', 'r', 'LineWidth', 1.2);
    Xd_paths = gobjects(1, defender);
    for i = 1:defender
        Xd_paths(i) = animatedline('Color', 'b', 'LineWidth', 1.2);
    end
    Xt_path = animatedline('Color', 'k', 'LineWidth', 1.2);

    % Add starting points
    Xa_init = xinit(1:3)';
    Xt_init = xinit(3*(defender+1)+1:3*(defender+2))';
    plot3(Xa_init(1), Xa_init(2), Xa_init(3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'a');
    text(Xa_init(1), Xa_init(2), Xa_init(3), 'a', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    for i = 1:defender
        Xd_init{i} = xinit(3*i+1:3*(i+1))';
        plot3(Xd_init{i}(1), Xd_init{i}(2), Xd_init{i}(3), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', sprintf('d%d', i));
        text(Xd_init{i}(1), Xd_init{i}(2), Xd_init{i}(3), sprintf('d%d', i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
    plot3(Xt_init(1), Xt_init(2), Xt_init(3), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 't');
    text(Xt_init(1), Xt_init(2), Xt_init(3), 't', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

    % Add starting points to animated lines
    addpoints(Xa_path, Xa_init(1), Xa_init(2), Xa_init(3));
    for i = 1:defender
        addpoints(Xd_paths(i), Xd_init{i}(1), Xd_init{i}(2), Xd_init{i}(3));
    end
    addpoints(Xt_path, Xt_init(1), Xt_init(2), Xt_init(3));

    % Simulation loop
    destroyed_defenders = false(1, defender);

    for k = 1:length(t)-1
        Xa = x(1:3,k+1)';
        for i = 1:defender
            if ~destroyed_defenders(i)
                Xd{i} = x(3*i+1:3*(i+1),k+1)';
            end
        end
        Xt = x(3*(defender+1)+1:3*(defender+2),k+1)';
        
        addpoints(Xa_path, Xa(1), Xa(2), Xa(3));
        for i = 1:defender
            if ~destroyed_defenders(i)
                addpoints(Xd_paths(i), Xd{i}(1), Xd{i}(2), Xd{i}(3));
            end
        end
        addpoints(Xt_path, Xt(1), Xt(2), Xt(3));

        if ~capture
            break;
        end
        drawnow
        pause(0.06);
    end

    if capture
        disp("No outcome");
    end

    if ~capture 
        if strcmp(capture_type, 'attacker')
            r = par.sigma(1);
            [X, Y, Z] = sphere(50);
            X = X * r; Y = Y * r; Z = Z * r;
            surf(X + capture_position(1), Y + capture_position(2), Z + capture_position(3), 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'r');
            plot3(capture_position(1), capture_position(2), capture_position(3), 'ro', 'MarkerFaceColor', 'r');
        else
            r = par.sigma(2);
            [X, Y, Z] = sphere(50);
            X = X * r; Y = Y * r; Z = Z * r;
            surf(X + capture_position(1), Y + capture_position(2), Z + capture_position(3), 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'b');
            plot3(capture_position(1), capture_position(2), capture_position(3), 'bo', 'MarkerFaceColor', 'b');
        end
        shading interp;  % Interpolate shading for a smooth appearance
        light;  % Add a default light
    end
end
