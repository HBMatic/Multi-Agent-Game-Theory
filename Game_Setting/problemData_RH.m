function par=  problemData_RH(Xa,Xt)
    par.sigma(1)=0.17;
    par.kappa=2;
    par.psi=norm(Xa-Xt)-par.kappa*par.sigma(1);
    par.deltaRH=0.5;
    par.t0=0;
   
end