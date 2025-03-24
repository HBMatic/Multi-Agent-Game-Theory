function [Xa_path, Xd_paths, Xt_path] = initializePlots_2D(xinit, defender)
    Xa_path = animatedline('Color', 'r', 'LineWidth', 1.5);
    Xd_paths = gobjects(1, defender);
    for i = 1:defender
        Xd_paths(i) = animatedline('Color', 'b', 'LineWidth', 1.5);
    end
    Xt_path = animatedline('Color', 'k', 'LineWidth', 1.5);

    Xa_init = xinit(1:2)';
    Xt_init = xinit(2*(defender+1)+1:2*(defender+2))';
    plot(Xa_init(1), Xa_init(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'a');
    text(Xa_init(1), Xa_init(2), 'a', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    for i = 1:defender
        Xd_init{i} = xinit(2*i+1:2*(i+1))';
        plot(Xd_init{i}(1), Xd_init{i}(2), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', sprintf('d%d', i));
        text(Xd_init{i}(1), Xd_init{i}(2), sprintf('d%d', i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
    plot(Xt_init(1), Xt_init(2), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 't');
    text(Xt_init(1), Xt_init(2), 't', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

    addpoints(Xa_path, Xa_init(1), Xa_init(2));
    for i = 1:defender
        addpoints(Xd_paths(i), Xd_init{i}(1), Xd_init{i}(2));
    end
    addpoints(Xt_path, Xt_init(1), Xt_init(2));
end