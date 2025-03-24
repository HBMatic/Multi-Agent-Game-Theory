function updatePlots_3D(Xa_path, Xd_paths, Xt_path, Xa, Xd, Xt, defender, destroyed_defenders)
    addpoints(Xa_path, Xa(1), Xa(2), Xa(3));
    for i = 1:defender
        if ~destroyed_defenders(i)
            addpoints(Xd_paths(i), Xd{i}(1), Xd{i}(2), Xd{i}(3));
        end
    end
    addpoints(Xt_path, Xt(1), Xt(2), Xt(3));
    drawnow;
end