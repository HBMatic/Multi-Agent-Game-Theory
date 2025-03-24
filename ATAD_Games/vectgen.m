function vect=vectgen(mat)
%
N=length(mat); 
%
vect=[];
for k=1:N
    vect=[vect mat(k,k:N)];
end
vect=vect';
end
    