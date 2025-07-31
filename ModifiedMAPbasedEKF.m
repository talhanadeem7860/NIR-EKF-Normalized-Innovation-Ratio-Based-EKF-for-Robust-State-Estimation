function[x_est_MAP] =ModifiedMAPbasedEKF(F,Q,x0,T,m,ynoisy,R)
y_mean_MAP = zeros(m,T);
% S_MAP = zeros(1,1,2);
% K_MAP = zeros(4,1,2);
% x_aposteriori_current_given_outliers_MAP = zeros(4,2);
% P_aposteriori_current_given_outliers_MAP = zeros(4,4,2);
P_2_store=[];
covariance_outlier_est_MAP = [10^10 0];

x_aposteriori_previous_MAP=x0;
P_aposteriori_previous_MAP=Q;
x_est_MAP = [];
for i = 1:T
P_estimate=F*P_aposteriori_previous_MAP*F'+ Q;
x_estimate=F*x_aposteriori_previous_MAP;


for k = 1:m
    
    x = x_estimate(1);
    y = x_estimate(3);

    

    if(mod(k,2) ==0)
    t=k/2;
    y_mean_MAP(k,i)=sqrt((x-(t-1)*350)^2+(y-(350*mod(t-1,2)))^2);
    else
    t=ceil(k/2);    
    y_mean_MAP(k,i)=atan2(y-(350*(mod(t,2))),x-(t-1)*350);
    end
    
for kk = 1:2
Hmod = eval_jacob1(2*k,x,y);
H = Hmod(k,:);
S_MAP(:,:,kk)= H*P_estimate*H' + covariance_outlier_est_MAP(:,kk) + R(k,k);
K_MAP(:,:,kk) = P_estimate*H'*(S_MAP(:,:,kk))^-1;
x_aposteriori_current_given_outliers_MAP(:,kk)=x_estimate + K_MAP(:,:,kk) * (ynoisy(k,i) - y_mean_MAP(k,i));
P_aposteriori_current_given_outliers_MAP(:,:,kk) = P_estimate - K_MAP(:,:,kk)*S_MAP(:,:,kk)*K_MAP(:,:,kk)';

P_aposteriori_current_given_outliers_MAP(:,:,kk)=(P_aposteriori_current_given_outliers_MAP(:,:,kk)+P_aposteriori_current_given_outliers_MAP(:,:,kk)')/2; % To ensure symmetry despite Round Off Errors

S_MAP(:,:,kk)=(S_MAP(:,:,kk)+S_MAP(:,:,kk)')/2; % To ensure symmetry despite Round Off Errors
end
probs=[];
% for kk=1:2 
% sum_=0;
% for j=1:2
% % sum_ =sum_+v_probVec(x_aposteriori_current_given_outliers_MAP(:,kk),x_aposteriori_current_given_outliers_MAP(:,j),P_aposteriori_current_given_outliers_MAP(:,:,j))*normpdf(ynoisy(k,i),y_mean_MAP(k,i),sqrt(S_MAP(:,:,j))); 
% end
% probs = [probs sum_]
% end


for j=1:2 
sum_=0;
% sum_ =sum_+v_probVec(x_aposteriori_current_given_outliers_MAP(:,kk),x_aposteriori_current_given_outliers_MAP(:,j),P_aposteriori_current_given_outliers_MAP(:,:,j))*normpdf(ynoisy(k,i),y_mean_MAP(k,i),sqrt(S_MAP(:,:,j))); 
sum_ =sum_+normpdf(ynoisy(k,i),y_mean_MAP(k,i),sqrt(S_MAP(:,:,j))); 
probs = [probs sum_];
end

[~, idx_]=max(probs);

x_posteriori=x_aposteriori_current_given_outliers_MAP(:,idx_);
P_aposteriori_current_MAP=P_aposteriori_current_given_outliers_MAP(:,:,idx_);
P_estimate=P_aposteriori_current_MAP;

x_estimate=x_posteriori;

end


x_aposteriori_previous_MAP = x_estimate;
P_aposteriori_previous_MAP = P_estimate;



x_est_MAP=[x_est_MAP x_posteriori];


P_2_store=cat(3, P_2_store , P_estimate);


end


end
