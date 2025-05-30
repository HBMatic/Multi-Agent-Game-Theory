function par=  problemData_RH(Xa,Xt)
    par.sigma(1)=0.3;
    par.kappa=3;
    par.psi=norm(Xa-Xt)-par.kappa*par.sigma(1);
    par.deltaRH=1;
    par.t0=0;
   
end