function mat=symgen(vect)
%
N=0.5*(sqrt(1+8*length(vect))-1); 
%
vect=vect';
mat(1,:)=vect(1:N); iInd=N; mat(1,1)=0.5*mat(1,1);
for k=2:N     
    mat(k,:)=[zeros(1,k-1) vect(iInd+1:iInd+N-k+1)];
    iInd=iInd+N-k+1;
    mat(k,k)=0.5*mat(k,k);
end
%
mat=(mat+mat');
%
    