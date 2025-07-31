function[Jacob] = eval_jacob(m,a,b) 
% Jacob = [(a - 10)*inv(sqrt((a - 10)^2 + (b - 20)^2)) (b - 20)*inv(sqrt((a - 10)^2 + (b - 20)^2));
%           (a - 30)*inv(sqrt((a - 30)^2 + (b - 40)^2)) (b - 40)*inv(sqrt((a - 30)^2 + (b - 40)^2))];


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
zc = zeros(m,1);
Jacob = [s, zc];


% ans = subs(newmatrix,[a b],xdash);
% Jacob = vpa(ans,2);
end