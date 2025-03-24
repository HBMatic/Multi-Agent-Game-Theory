clear all; 
T = 200;     
delta = 0.2; 
t = 0:delta:T;
w_param = 3;  
h_param = 2; 
x(1)=0;
y(1)=0;
for k=1:length(t)
    x(k+1)=x(k)+delta*(w_param*pi/T)*sqrt(1-(2*x(k)/w_param)^2);
    y(k+1)=y(k)+delta*(2*h_param*pi/T)*sqrt(1-(2*y(k)/h_param)^2);
end