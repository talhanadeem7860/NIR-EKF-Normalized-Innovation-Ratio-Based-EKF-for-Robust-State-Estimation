function[Jacob] = eval_jacob(m,a,b) 
for i =1:m/2
    n=2*i-1;
     
    
    s(n,1)=-(b-(350*(mod(i,2))))/(((b-(350*(mod(i,2))))^2+(a-(i-1)*350)^2));
    %final_s = diag(s);
   
end

for i =1:m/2
    n=2*i-1;
     
    
    s(n,3)=(a-(i-1)*350)/(((b-(350*(mod(i,2)))).^2+(a-(i-1)*350).^2));
    %final_s = diag(s);
    
end

    
for i=1:m/2
    n=2*i;
    s(n,1)=((a-(i-1)*350))/sqrt((a-(i-1)*350).^2+(b-(350*mod(i+1,2))).^2);
    %final_s = diag(s);
end

for i=1:m/2
    n=2*i;
    s(n,3)=(b-(350*mod(i+1,2)))/sqrt((a-(i-1)*350).^2+(b-(350*mod(i+1,2))).^2);
    %final_s = diag(s);
end
zc = zeros(m,2);
Jacob = [s, zc];
% ans = subs(newmatrix,[a b],xdash);
% Jacob = vpa(ans,2);
end