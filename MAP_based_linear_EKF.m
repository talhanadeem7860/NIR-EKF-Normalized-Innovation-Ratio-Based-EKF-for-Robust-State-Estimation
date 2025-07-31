close all
Trmse_1=[];
Trmse_2=[];
Trmse_3=[];

Dimension=2;
outlier_independence=0;

for lambda = 0:0.02:0.5
delT = 1;             % sampling Time
F=[1 delT 0 0;0 1 0 0;0 0 1 delT;0 0 0 1];    

alpha = 1000;

mean_observation_outlier=[0 0 0 0];

% covariance_observation_outlier=5*eye(4);
% covariance_observation_outlier(3,3) = 10^-3;
% covariance_observation_outlier(4,4) = 10^-3;

L =10;  
Position_diff=[];
Position_diff_MAP=[];
Velocity_diff = [];
Velocity_diff_MAP = [];
for l = 1:L


dt=1;
% total_time = 1000;
% t =1:total_time;        % Time
T = 1000;
nstates = 4;         % numbers of States
nmeas = 4;           % number of measurements 
% F = @(x)[1 sin(x(5)*dt)/x(5) 0 (cos(x(5)*dt)-1)/x(5) (dt*x(2)*cos(dt*x(5))/x(5) - (x(4)*cos(dt*x(5)-1))/x(5)^2 -(x(2)*sin(dt*x(5))/x(5)^2)-(dt*x(4)*sin(dt*x(5))/x(5)));0 cos(x(5)*dt) 0 -sin(x(5)*dt) -dt*sin(x(5)*dt)*x(2)-dt*cos(x(5)*dt)*x(4);0 -(cos(x(5)*dt-1)/x(5)) 1 (sin(x(5)*dt))/x(5) (x(2)*(cos(dt*x(5)-1))/x(5)^2 - (x(4)*sin(dt*x(5))/x(5)^2)+(dt*x(4)*cos(dt*x(5))/x(5)+(dt*x(4)*sin(dt*x(5))/x(5))));0 sin(x(5)*dt) 0 cos(x(5)*dt) dt*x(2)*cos(x(5)*dt)-dt*x(4)*sin(dt*x(5));0 0 0 0 1];    
% T=length(t);
% R= [10 0 0 0;0 10 0 0;0 0 1.6*10^-3 0;0 0 0 1.6*10^-3];
R= 5*eye(4);

R(3,3)=10^-3;
R(4,4)=10^-3;
Q= .1*eye(4);


P0 = Q; % Set initial error covariance
% noise =[mvnrnd([0 0 0 0],R,total_time)]';

% x0=[-10000;10;5000;-5;-0.053];
x0=[300;50;100;10];
xt= x0;
% xt(:,1) =  x0+ mvnrnd([0 0 0 0 0],Q)' ; % Set true initial state
% f1=@(x)[[1 sin(x(5)*dt)/x(5) 0 (cos(x(5)*dt)-1)/x(5) 0 ; 0 cos(x(5)*dt) 0 -sin(x(5)*dt) 0; 0 (1-cos(x(5)*dt))/x(5) 1 sin(x(5)*dt)/x(5) 0; 0 sin(x(5)*dt) 0 cos(x(5)*dt) 0;0 0 0 0 1] * x];
MAP_prob_vec=[];


outlier_store=[];
vt_store=[];
prob_diff=[];
xtr=[];
% ynoisy = zeros(4, T); % Initialize size of output vector for all k
for k = 1:T
    
    xt(:,k) = F*xt+[mvnrnd([0 0 0 0],Q)]';
    xt = xt(:,k);
    event=rand;
    
    if event < lambda 
   
     
    if Dimension==1    
    
        
        observation_outlier = [mvnrnd([0],alpha*R(1,1))  0 0 0];
    
    
    elseif Dimension == 2 
        
        
        observation_outlier = [0 mvnrnd([0],alpha*R(2,2)) 0 0];
    
    elseif Dimension == 3
        
        observation_outlier = [0 0 mvnrnd([0],alpha*R(3,3)) 0];
    
    elseif Dimension == 4
        
        observation_outlier = [0 0 0 mvnrnd([0],alpha*R(4,4))];
        
        
        
        
    end
    
    
    else
        
       observation_outlier=[0 0 0 0]; 
        
    end
    
    outlier_store=[outlier_store observation_outlier'];

%     if Dimension==1
%       
%         vt1=[(event>=lambda)*mvnrnd([0],R(1,1))+observation_outlier(1) mvnrnd([0],R(2,2))+observation_outlier(2) mvnrnd([0],R(3,3))+observation_outlier(3) mvnrnd([0],R(4,4))+observation_outlier(4)];
%      
%     elseif Dimension ==2
% 
%         vt1=[mvnrnd([0],R(1,1))+observation_outlier(1) (event>=lambda)*mvnrnd([0],R(2,2))+observation_outlier(2) mvnrnd([0],R(3,3))+observation_outlier(3) mvnrnd([0],R(4,4))+observation_outlier(4)];
%     
%     elseif Dimension ==3    
%         
%         vt1=[mvnrnd([0],R(1,1))+observation_outlier(1) mvnrnd([0],R(2,2))+observation_outlier(2) (event>=lambda)*mvnrnd([0],R(3,3))+observation_outlier(3) mvnrnd([0],R(4,4))+observation_outlier(4)];
%      
%     elseif Dimension == 4
%         
%         vt1=[mvnrnd([0],R(1,1))+observation_outlier(1) mvnrnd([0],R(2,2))+observation_outlier(2) mvnrnd([0],R(3,3))+observation_outlier(3) (event>=lambda)*mvnrnd([0],R(4,4))+observation_outlier(4)];
%     end
    
%     vt_store=[vt_store vt1'];
    
            ynoisy(:,k) = [sqrt(xt(1)^2 + xt(3)^2);sqrt((xt(1)-350)^2 + (xt(3)-350)^2); ...
          atan2d((xt(3)-350),xt(1));
            atan2d(xt(3),xt(1)-350)]+observation_outlier'+mvnrnd([0 0 0 0],R)';
            

%     y(:,t)=[sqrt(x_t(1)^2+x_t(3)^2);atan2(x_t(3),x_t(1))]+vt1';
    
    
 
xtr = [xtr xt];
end

% for k = 1:T
%     ytrue(:,k) = [sqrt(xt(1,k)^2 + xt(3,k)^2);sqrt((xt(1,k)-350)^2 + (xt(3,k)-350)^2); ...
%          atan2d((xt(3,k)-350),xt(1,k));
%            atan2d(xt(3,k),xt(1,k)-350)]; 
% ynoisy(:,k) = ytrue(:,k) + noise(:,k);           
% end

% for k = 1:T
%     ytrue(:,k) = [sqrt(xt(1)^2 + xt(3)^2);sqrt((xt(1)-350)^2 + (xt(3)-350)^2); ...
%          sqrt((xt(1)-0)^2 + (xt(3)-350)^2);
%           sqrt((xt(1)-350)^2 + (xt(3)-0)^2)] ; 
% ynoisy(:,k) = ytrue(:,k) + noise(:,k);           
end
%Using Kalman Filtering

% P_1_store=[];
% lambda_est=[0.2 0.2]';
% 
% if outlier_independence==0
%     pi_vec_est=[1-sum(lambda_est) lambda_est']; % Pr([No outlier, outlier in first dim, outlier in second dim])
% 
% else
%     pi_vec_est=[1-sum(lambda_est)+lambda_est(1)*lambda_est(2) lambda_est']; % Pr([No outlier, outlier in first dim, outlier in second dim])
% 
%    
% end
% mean_outlier_vec_est=[0 0 0]; % Third_Dim is in for no outlier
% covariance_outlier_est(:,:,1)=500*eye(4);
% covariance_outlier_est(:,:,2)=5*eye(4);
% 
% 

% 
% x_est=x0;
% P_=Q;
% 
% x_est=[];
% P_1_store=[];
% for i=1:T
%     
% 
% P_apriori_current=F*P_*F'+ Q;
% x_apriori_current=F*x_est;
% 
% x = x_apriori_current(1);
% y = x_apriori_current(3);
%   H=[x/(x^2 + y^2)^(1/2),0,y/(x^2 + y^2)^(1/2),0;(x-350)/((x-350)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-350)^2 + (y-350)^2)^(1/2),0;
%    -(y-350)/(x^2+(y-350)^2),0,x/(x^2+(y-350)^2),0;y/((x-350)^2+y^2),0,(x-350)/((x-350)^2+y^2),0];
% 
% y_est(:,i) = [sqrt(x^2 + y^2);sqrt((x-350)^2 + (y-350)^2);atan2d((y-350),x);atan2d(y,(x-350))];
% 
% S=(H*P_apriori_current*H' + R);
% K = P_apriori_current*H'*inv(S); % Calculate Kalman gain
% x_est(:,i) = x_apriori_current + K*(ynoisy(:,i) - y_est(:,i)); % Update state estimate
% P_ = (eye(nstates)-K*H)*P_apriori_current; % Update covariance estimate
% 
% 
% % Updating values for next iteration
% x_est=[x_est x_est];
% % x_aposteriori_previous = x_aposteriori_current ;
% % P_aposteriori_previous=P_aposteriori_current;
% 
% P_1_store=cat(3, P_1_store ,  P_);

% Prediction
x_est = x0;
P = P0;
for k = 1:T
if k==1
x_est(:,1) = x0;
P = P0;
else
x_pred = F*x_est(:,k-1);
P_pred = F*P*F'+Q;
 x=x_pred(1);
 y=x_pred(3);
 
y_est(:,k) = [sqrt(x_pred(1)^2 + x_pred(3)^2);sqrt((x_pred(1)-350)^2 + (x_pred(3)-350)^2);atan2d((x_pred(3)-350),x_pred(1));atan2d(x_pred(3),(x_pred(1)-350))];
 
%%% Jacobian Matrix  %%%%%
 H=[x/(x^2 + y^2)^(1/2),0,y/(x^2 + y^2)^(1/2),0;(x-350)/((x-350)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-350)^2 + (y-350)^2)^(1/2),0;
  -(y-350)/(x^2+(y-350)^2),0,x/(x^2+(y-350)^2),0;  y/((x-350)^2+y^2),0,(x-350)/((x-350)^2+y^2),0];


% Measurement Update
S=(H*P_pred*H' + R);
K = P_pred*H'*inv(S); % Calculate Kalman gain
x_est(:,k) = x_pred + K*(ynoisy(:,k) - y_est(:,k)); % Update state estimate
P = (eye(nstates)-K*H)*P_pred; % Update covariance estimate
% h(:,k)=[sqrt(x_est(1,k)^2 + x_est(3,k)^2);sqrt((x_est(1,k)-350)^2 + (x_est(3,k)-350)^2); ...
%          atan2d((x_est(3,k)-350),x_est(1,k));
%            atan2d(x_est(3,k),x_est(1,k)-350)];






% end
end

end



%MAP estimation
P_2_store=[];
lambda_est_MAP=[0.2 0.2]';

if outlier_independence==0
    pi_vec_est_MAP=[1-sum(lambda_est_MAP) lambda_est_MAP']; % Pr([No outlier, outlier in first dim, outlier in second dim])

else
    pi_vec_est_MAP=[1-sum(lambda_est_MAP)+lambda_est_MAP(1)*lambda_est_MAP(2) lambda_est_MAP']; % Pr([No outlier, outlier in first dim, outlier in second dim])

   
end
mean_outlier_vec_est_MAP=[0 0 0]; % Third_Dim is in for no outlier
covariance_outlier_est_MAP(:,:,1)=500*eye(4);
covariance_outlier_est_MAP(:,:,2)=5*eye(4);




x_aposteriori_previous_MAP=x0;
P_aposteriori_previous_MAP=Q;

x_est_MAP=[];

for i=1:T

        
    
P_apriori_current_MAP=F*P_aposteriori_previous_MAP*F'+ Q;
x_apriori_current_MAP=F*x_aposteriori_previous_MAP;

x = x_apriori_current_MAP(1);
y = x_apriori_current_MAP(3);
H=[x/(x^2 + y^2)^(1/2),0,y/(x^2 + y^2)^(1/2),0;(x-350)/((x-350)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-350)^2 + (y-350)^2)^(1/2),0;
   -(y-350)/(x^2+(y-350)^2),0,x/(x^2+(y-350)^2),0;y/((x-350)^2+y^2),0,(x-350)/((x-350)^2+y^2),0];



% H = [ x_apriori_current_MAP(1)/(x_apriori_current_MAP(1)^2 + x_apriori_current_MAP(3)^2)^(1/2), 0, x_apriori_current_MAP(3)/(x_apriori_current_MAP(1)^2 + x_apriori_current_MAP(3)^2)^(1/2), 0, 0; -(imag(x_apriori_current_MAP(1)) + real(x_apriori_current_MAP(3)))/((imag(x_apriori_current_MAP(1)) + real(x_apriori_current_MAP(3)))^2 + (imag(x_apriori_current_MAP(3)) - real(x_apriori_current_MAP(1)))^2), 0, -(imag(x_apriori_current_MAP(3)) - real(x_apriori_current_MAP(1)))/((imag(x_apriori_current_MAP(1)) + real(x_apriori_current_MAP(3)))^2 + (imag(x_apriori_current_MAP(3)) - real(x_apriori_current_MAP(1)))^2), 0, 0];


y_mean_MAP(:,1) = [sqrt(x^2 + y^2);sqrt((x-350)^2 + (y-350)^2);atan2d((y-350),x);atan2d(y,(x-350))] + mean_outlier_vec_est_MAP(1);
y_mean_MAP(:,2) = [sqrt(x^2 + y^2);sqrt((x-350)^2 + (y-350)^2);atan2d((y-350),x);atan2d(y,(x-350))] + mean_outlier_vec_est_MAP(2);
y_mean_MAP(:,3) = [sqrt(x^2 + y^2);sqrt((x-350)^2 + (y-350)^2);atan2d((y-350),x);atan2d(y,(x-350))] + mean_outlier_vec_est_MAP(3);


S_MAP(:,:,1)= H*P_apriori_current_MAP*H' + covariance_outlier_est_MAP(:,:,1) + R; % For Outlier in First Dim
S_MAP(:,:,2)= H*P_apriori_current_MAP*H' + covariance_outlier_est_MAP(:,:,2) + R; % For Outlier in Second Dim
S_MAP(:,:,3)= H*P_apriori_current_MAP*H' + R; % In case of No outlier


K_MAP(:,:,1) = P_apriori_current_MAP*H'*(S_MAP(:,:,1))^-1; % Outlier in Dim 1
K_MAP(:,:,2) = P_apriori_current_MAP*H'*(S_MAP(:,:,2))^-1; % Outlier in Dim 2
K_MAP(:,:,3) = P_apriori_current_MAP*H'*(S_MAP(:,:,3))^-1; % No Outlier


x_aposteriori_current_given_outliers_MAP(:,1)=x_apriori_current_MAP + K_MAP(:,:,1) * (ynoisy(:,i) - y_mean_MAP(:,1)); % Outlier in Dim 1
x_aposteriori_current_given_outliers_MAP(:,2)=x_apriori_current_MAP + K_MAP(:,:,2) * (ynoisy(:,i) - y_mean_MAP(:,2)); % Outlier in Dim 2
x_aposteriori_current_given_outliers_MAP(:,3)=x_apriori_current_MAP + K_MAP(:,:,3) * (ynoisy(:,i) - y_mean_MAP(:,3)); % No Outlier

P_aposteriori_current_given_outliers_MAP(:,:,1) = P_apriori_current_MAP - K_MAP(:,:,1)*S_MAP(:,:,1)*K_MAP(:,:,1)'; % Outlier in Dim 1
P_aposteriori_current_given_outliers_MAP(:,:,2) = P_apriori_current_MAP - K_MAP(:,:,2)*S_MAP(:,:,2)*K_MAP(:,:,2)'; % Outlier in Dim 2
P_aposteriori_current_given_outliers_MAP(:,:,3) = P_apriori_current_MAP - K_MAP(:,:,3)*S_MAP(:,:,3)*K_MAP(:,:,3)'; % No Outlier

P_aposteriori_current_given_outliers_MAP(:,:,1)=(P_aposteriori_current_given_outliers_MAP(:,:,1)+P_aposteriori_current_given_outliers_MAP(:,:,1)')/2; % To ensure symmetry despite Round Off Errors
P_aposteriori_current_given_outliers_MAP(:,:,2)=(P_aposteriori_current_given_outliers_MAP(:,:,2)+P_aposteriori_current_given_outliers_MAP(:,:,2)')/2; % To ensure symmetry despite Round Off Errors
P_aposteriori_current_given_outliers_MAP(:,:,3)=(P_aposteriori_current_given_outliers_MAP(:,:,3)+P_aposteriori_current_given_outliers_MAP(:,:,3)')/2; % To ensure symmetry despite Round Off Errors

S_MAP(:,:,1)=(S_MAP(:,:,1)+S_MAP(:,:,1)')/2; % To ensure symmetry despite Round Off Errors
S_MAP(:,:,2)=(S_MAP(:,:,2)+S_MAP(:,:,2)')/2; % To ensure symmetry despite Round Off Errors
S_MAP(:,:,3)=(S_MAP(:,:,3)+S_MAP(:,:,3)')/2; % To ensure symmetry despite Round Off Errors


% Evaluating Probabilities (state given measurement)

probability_mk_00_MAP = pi_vec_est_MAP(1)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,3),x_aposteriori_current_given_outliers_MAP(:,3),P_aposteriori_current_given_outliers_MAP(:,:,3))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,3),S_MAP(:,:,3)) ...
                  + pi_vec_est_MAP(2)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,3),x_aposteriori_current_given_outliers_MAP(:,1),P_aposteriori_current_given_outliers_MAP(:,:,1))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,1),S_MAP(:,:,1)) ...
                  + pi_vec_est_MAP(3)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,3),x_aposteriori_current_given_outliers_MAP(:,2),P_aposteriori_current_given_outliers_MAP(:,:,2))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,2),S_MAP(:,:,2));

probability_mk_01_MAP = pi_vec_est_MAP(1)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,1),x_aposteriori_current_given_outliers_MAP(:,3),P_aposteriori_current_given_outliers_MAP(:,:,3))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,3),S_MAP(:,:,3)) ...
                     + pi_vec_est_MAP(2)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,1),x_aposteriori_current_given_outliers_MAP(:,1),P_aposteriori_current_given_outliers_MAP(:,:,1))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,1),S_MAP(:,:,1)) ...
                     + pi_vec_est_MAP(3)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,1),x_aposteriori_current_given_outliers_MAP(:,2),P_aposteriori_current_given_outliers_MAP(:,:,2))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,2),S_MAP(:,:,2));

probability_mk_02_MAP = pi_vec_est_MAP(1)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,2),x_aposteriori_current_given_outliers_MAP(:,3),P_aposteriori_current_given_outliers_MAP(:,:,3))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,3),S_MAP(:,:,3)) ...
                     + pi_vec_est_MAP(2)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,2),x_aposteriori_current_given_outliers_MAP(:,1),P_aposteriori_current_given_outliers_MAP(:,:,1))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,1),S_MAP(:,:,1)) ...
                     + pi_vec_est_MAP(3)*mvnpdf(x_aposteriori_current_given_outliers_MAP(:,2),x_aposteriori_current_given_outliers_MAP(:,2),P_aposteriori_current_given_outliers_MAP(:,:,2))*mvnpdf(ynoisy(:,i),y_mean_MAP(:,2),S_MAP(:,:,2));

                 
MAP_prob_vec=[MAP_prob_vec [probability_mk_00_MAP probability_mk_01_MAP probability_mk_02_MAP ]'];
if max([probability_mk_00_MAP probability_mk_01_MAP probability_mk_02_MAP ])== probability_mk_00_MAP

x_aposteriori_current_MAP = x_aposteriori_current_given_outliers_MAP(:,3);
P_aposteriori_current_MAP = P_aposteriori_current_given_outliers_MAP(:,:,3);

elseif max([probability_mk_00_MAP probability_mk_01_MAP probability_mk_02_MAP ])== probability_mk_01_MAP

x_aposteriori_current_MAP = x_aposteriori_current_given_outliers_MAP(:,1);
P_aposteriori_current_MAP = P_aposteriori_current_given_outliers_MAP(:,:,1);

else

x_aposteriori_current_MAP = x_aposteriori_current_given_outliers_MAP(:,2);
P_aposteriori_current_MAP = P_aposteriori_current_given_outliers_MAP(:,:,2);

    
end

% Updating values for next iteration
x_est_MAP=[x_est_MAP x_aposteriori_current_MAP];
x_aposteriori_previous_MAP = x_aposteriori_current_MAP;
P_aposteriori_previous_MAP=P_aposteriori_current_MAP;

P_2_store=cat(3, P_2_store , P_aposteriori_previous_MAP);


end




% 
% 
% xest = zeros(4,T);
% h(:,1)=ynoisy(:,1);
% 
% x0;
% P = P0; 
% % x_pred = f1(x0);
% % P_pred = F(x0)*P*F(x0)'+Q;
% x_pred=x0;
% P_pred=P;
% x=x_pred(1);
%  y=x_pred(3);
%  
% 
% y_est(:,1) = [sqrt(x_pred(1)^2 + x_pred(3)^2);sqrt((x_pred(1)-350)^2 + (x_pred(3)-350)^2);sqrt((x_pred(1))^2 + (x_pred(3)-350)^2);sqrt((x_pred(1)-350)^2 + (x_pred(3)-0)^2)];
% 
%  H=[x/(x^2 + y^2)^(1/2),0,y/(x^2 + y^2)^(1/2),0,0;(x-350)/((x-350)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-350)^2 + (y-350)^2)^(1/2),0,0;
%   (x-0)/((x-0)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-0)^2 + (y-350)^2)^(1/2),0,0; (x-350)/((x-350)^2 + (y-0)^2)^(1/2),0,(y-0)/((x-350)^2 + (y-0)^2)^(1/2),0,0];
% 
% S=(H*P_pred*H' + R);
% K = P_pred*H'*inv(S); % Calculate Kalman gain
% err(:,1)=ynoisy(:,1) - y_est(:,1);
% xest(:,1) = x_pred + K*(ynoisy(:,1) - y_est(:,1)); % Update state estimate
% P = (eye(nstates)-K*H)*P_pred; 
% 
% 
% for k = 2:T
% % Prediction
% 
% x_pred = f1(xest(:,k-1));
% P_pred = F(xest(:,k-1))*P*F(xest(:,k-1))'+Q;
%  x=x_pred(1);
%  y=x_pred(3);
%  
% % Observation
% % y_est(:,k) = [sqrt(x_pred(1)^2 + x_pred(3)^2);sqrt((x_pred(1)-350)^2 + (x_pred(3)-350)^2);atan2d((x_pred(3)-350),x_pred(1));atan2d(x_pred(3),(x_pred(1)-350))];
% %  %%% Jacobian Matrix  %%%%%
% %  H=[x/(x^2 + y^2)^(1/2),0,y/(x^2 + y^2)^(1/2),0;(x-350)/((x-350)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-350)^2 + (y-350)^2)^(1/2),0;
% %   -(y-350)/(x^2+(y-350)^2),0,x/(x^2+(y-350)^2),0;  y/((x-350)^2+y^2),0,(x-350)/((x-350)^2+y^2),0];
% 
% y_est(:,k) = [sqrt(x_pred(1)^2 + x_pred(3)^2);sqrt((x_pred(1)-350)^2 + (x_pred(3)-350)^2);sqrt((x_pred(1))^2 + (x_pred(3)-350)^2);sqrt((x_pred(1)-350)^2 + (x_pred(3)-0)^2)];
% 
% %%% Jacobian Matrix  %%%%%
% %  H=[x/(x^2 + y^2)^(1/2),0,y/(x^2 + y^2)^(1/2),0,0;(x-350)/((x-350)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-350)^2 + (y-350)^2)^(1/2),0,0;
% %   -(y-350)/(x^2+(y-350)^2),0,x/(x^2+(y-350)^2),0,0;y/((x-350)^2+y^2),0,(x-350)/((x-350)^2+y^2),0,0];
% 
%  H=[x/(x^2 + y^2)^(1/2),0,y/(x^2 + y^2)^(1/2),0,0;(x-350)/((x-350)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-350)^2 + (y-350)^2)^(1/2),0,0;
%   (x-0)/((x-0)^2 + (y-350)^2)^(1/2),0,(y-350)/((x-0)^2 + (y-350)^2)^(1/2),0,0; (x-350)/((x-350)^2 + (y-0)^2)^(1/2),0,(y-0)/((x-350)^2 + (y-0)^2)^(1/2),0,0];
% % x
% % y
% % H
% % Measurement Update
% S=(H*P_pred*H' + R);
% K = P_pred*H'*inv(S); % Calculate Kalman gain
% err(:,k)=ynoisy(:,k) - y_est(:,k);
% xest(:,k) = x_pred + K*(ynoisy(:,k) - y_est(:,k)); % Update state estimate
% P = (eye(nstates)-K*H)*P_pred; % Update covariance estimate
% % h(:,k)=[sqrt(xest(1,k)^2 + xest(3,k)^2);sqrt((xest(1,k)-350)^2 + (xest(3,k)-350)^2); ...
% %          atan2d((xest(3,k)-350),xest(1,k));
% %            atan2d(xest(3,k),xest(1,k)-350)];
% % P=.5*(P+P');
% PP(k) = P(1,1);

%% Calculating Time averaged root mean square (TRMSE)
diff=xtr-x_est;
Position_diff=cat(3, Position_diff,[diff(1,:);diff(3,:)]);
Velocity_diff = cat(3,Velocity_diff,[diff(2,:);diff(4,:)]);

diff_MAP=xtr-x_est_MAP;
Position_diff_MAP=cat(3, Position_diff_MAP,[diff_MAP(1,:);diff_MAP(3,:)]);
Velocity_diff_MAP = cat(3,Velocity_diff_MAP,[diff_MAP(2,:);diff_MAP(4,:)]);

% %% Final TRMSE
TRMSE=1/T*sum(sqrt((1/L)*sum(sum(Position_diff.^2+Velocity_diff.^2),3)));
TRMSE_MAP=1/T*sum(sqrt((1/L)*sum(sum(Position_diff_MAP.^2+Velocity_diff_MAP.^2),3)));    
Trmse_1=[Trmse_1 TRMSE];
Trmse_2=[Trmse_2 TRMSE_MAP];

end

figure,
plot(1:T,xtr(1,:),'r')
hold on
plot(1:T,x_est(1,:),'b')
hold on
% plot(1:T,xtr(1,:),'r')
% hold on
plot(1:T,x_est_MAP(1,:),'g')
legend('True','Kalman','MAP')
title('Tracking for position')


% figure,
% plot(1:T,xtr(1,:),'r')
% hold on
% plot(1:T,x_est_MAP(1,:),'b')
% title('Tracking for position for MAP Kalman')
% 
figure,
plot(1:T,xtr(2,:),'r')
hold on
plot(1:T,x_est(2,:),'b')
hold on
plot(1:T,x_est_MAP(2,:),'g')
title('Tracking for velocity')
legend('True','Kalman','MAP')

% 
% figure,
% plot(1:T,xtr(2,:),'r')
% hold on
% plot(1:T,x_est_MAP(2,:),'b')
% title('Tracking for velocity for MAP Kalman')

figure,
plot(0:0.02:0.5,Trmse_1,'r');
hold on;
plot(0:0.02:0.5,Trmse_2,'b');
legend('TRMSE','TRMSE_{MAP}');
ylabel('TRMSE Vector');
xlabel('Lambda');
% legend('TRMSE Kalman','TRMSE MAP based Kalman')
