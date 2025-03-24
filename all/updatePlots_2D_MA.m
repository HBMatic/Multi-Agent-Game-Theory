function updatePlots_2D_MA(Xa_paths, Xd_paths, Xt_path, Xa, Xd, Xt, defender, attacker, destroyed_defenders, destroyed_attackers)
    for i = 1:attacker
        if ~destroyed_attackers(i)
            addpoints(Xa_paths(i), Xa{i}(1), Xa{i}(2));
        end
    end
    
    for i = 1:defender
        if ~destroyed_defenders(i)
            addpoints(Xd_paths(i), Xd{i}(1), Xd{i}(2));
        end
    end
    
    addpoints(Xt_path, Xt(1), Xt(2));
end
