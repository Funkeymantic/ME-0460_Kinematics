function [ val ] = Csub(i,j,k,n,xi,A,Mp)

val=0;
for l = max(i,j):n
    val=val+bracket(A(k,i).M*xi(:,i),xi(:,k))'*A(l,k).M'*Mp(l).M*A(l,j).M*xi(:,j) + xi(:,i)'*A(l,i).M'*Mp(l).M*A(l,k).M*bracket(A(k,j).M*xi(:,j),xi(:,k));
end
