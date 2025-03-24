function init = initializeHessians_MA(par, defender,attacker)
    init = [vectgen(par.Qa1)];
    for i=2:attacker
        init = [init;vectgen(par.(['Qa',num2str(i)]))];
    end
    for i = 1:defender
        init = [init; vectgen(par.(['Qd', num2str(i)]))];
    end
    init = [init; vectgen(par.Qt1)];
end