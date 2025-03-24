function plotCircle(x, y, r, edgeColor, centerColor)
    theta = linspace(0, 2*pi, 100);
    hold on;
    plot(x + r * cos(theta), y + r * sin(theta), edgeColor, 'LineWidth', 1.5);
    plot(x, y, 'o', 'MarkerEdgeColor', centerColor, 'MarkerFaceColor', centerColor);
end