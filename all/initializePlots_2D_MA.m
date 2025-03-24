function [Xa_paths, Xd_paths, Xt_path] = initializePlots_2D_MA(xinit, defender, attacker)
    Xa_paths = gobjects(1, attacker);
    for i = 1:attacker
        Xa_paths(i) = animatedline('Color', 'r', 'LineWidth', 1.5);
    end
    
    Xd_paths = gobjects(1, defender);
    for i = 1:defender
        Xd_paths(i) = animatedline('Color', 'b', 'LineWidth', 1.5);
    end
    
    Xt_path = animatedline('Color', 'k', 'LineWidth', 1.5);
    
    % Initialize and plot attacker positions
    for i = 1:attacker
        Xa_init{i} = xinit(2 * (i - 1) + 1 : 2 * i)';
        plot(Xa_init{i}(1), Xa_init{i}(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', sprintf('a%d', i));
        text(Xa_init{i}(1), Xa_init{i}(2), sprintf('a%d', i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
    
    % Initialize and plot defender positions
    for i = 1:defender
        Xd_init{i} = xinit(2 * (attacker + i - 1) + 1 : 2 * (attacker + i))';
        plot(Xd_init{i}(1), Xd_init{i}(2), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', sprintf('d%d', i));
        text(Xd_init{i}(1), Xd_init{i}(2), sprintf('d%d', i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
    
    % Initialize and plot target position
    Xt_init = xinit(2 * (attacker + defender) + 1 : 2 * (attacker + defender + 1))';
    plot(Xt_init(1), Xt_init(2), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 't');
    text(Xt_init(1), Xt_init(2), 't', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    % Add initial points to the animated lines
    for i = 1:attacker
        addpoints(Xa_paths(i), Xa_init{i}(1), Xa_init{i}(2));
    end
    
    for i = 1:defender
        addpoints(Xd_paths(i), Xd_init{i}(1), Xd_init{i}(2));
    end
    
    addpoints(Xt_path, Xt_init(1), Xt_init(2));
end
