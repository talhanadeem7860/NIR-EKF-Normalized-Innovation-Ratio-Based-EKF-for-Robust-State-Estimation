clear all
close all

Trmse_1=[];
Trmse_2=[];
Trmse_3=[];
elapsed_time_Kalman = [];
elapsed_time_MAP = [];
lambda = 0.2;
mm =2:2:2;
T_KALMAN = [];
T_MAP = [];
for m = mm
delT = 1;             % sampling Time
F=[1 delT 0 0;0 1 0 0;0 0 1 delT;0 0 0 1];    
alpha = 1000;
mean_observation_outlier=[0 0 0 0];

L =30;  

Position_diff=[];
Position_diff_MAP=[];
Velocity_diff = [];
Velocity_diff_MAP = []; 
for l = 1:L
% disp(l)    
dt=1;
T = 1000;
nstates = 4;         % numbers of States
R = zeros(m,m);
r1 = 10^-3;
r2 = 5;
for i=1:2:m
        R(i,i)= r1;
end
    
for i=2:2:m
        R(i,i)= r2;
end

Q= .1*eye(4);
P0 = Q; % Set initial error covariance
x0=[300;50;100;10];
xt= x0;
MAP_prob_vec=[];
outlier_store=[];
vt_store=[];
prob_diff=[];
xtr=[];
ynoisy = zeros(m,T);
observation_outlier = zeros(m,T);
for k = 1:T
    
        xt(:,k) = F*xt+[mvnrnd([0 0 0 0],Q)]';
    xt = xt(:,k);
    for i =1:m
    if rand < lambda
        observation_outlier(i,k) = mvnrnd(0,alpha*R(i,i));
    else
        observation_outlier(i,k) = mvnrnd(0,R(i,i));
    end    
    end    

    for i = 1:m/2
        n = 2*i-1;
        ynoisy(n,k)=atan2(xt(3)-(350*(mod(i,2))),xt(1)-(i-1)*350)+observation_outlier(n,k);
    end
    
    
    for i = 1:m/2
        n = 2*i;
        ynoisy(n,k)=sqrt((xt(1)-(i-1)*350)^2+(xt(3)-(350*mod(i-1,2)))^2)+observation_outlier(n,k);
    end
    
xtr = [xtr xt];
end

tic
%EKF

[x_est] = EKF_func(m,F,x0,P0,T,Q,R,ynoisy,nstates);

Time_KALMAN = toc;



%MAP estimation
tStart = tic;
% x_aposteriori_previous_MAP=x0;
% P_aposteriori_previous_MAP=Q;
% P_apriori_current_MAP=F*P_aposteriori_previous_MAP*F'+ Q;
% x_apriori_current_MAP=F*x_aposteriori_previous_MAP;

[x_est_MAP] =ModifiedMAPbasedEKF(F,Q,x0,T,m,ynoisy,R);
Time_MAP = toc(tStart); 

%% Calculating Time averaged root mean square (TRMSE)
diff=xtr-x_est;
Position_diff=cat(3, Position_diff,[diff(1,:);diff(3,:)]);
Velocity_diff = cat(3,Velocity_diff,[diff(2,:);diff(4,:)]);


diff_MAP=xtr-x_est_MAP;
Position_diff_MAP=cat(3, Position_diff_MAP,[diff_MAP(1,:);diff_MAP(3,:)]);
Velocity_diff_MAP = cat(3,Velocity_diff_MAP,[diff_MAP(2,:);diff_MAP(4,:)]);
end
% %% Final TRMSE
TRMSE=1/T*sum(sqrt((1/L)*sum(sum(Position_diff.^2+Velocity_diff.^2),3)));
TRMSE_MAP=1/T*sum(sqrt((1/L)*sum(sum(Position_diff_MAP.^2+Velocity_diff_MAP.^2),3)));    
Trmse_1=[Trmse_1 TRMSE];
Trmse_2=[Trmse_2 TRMSE_MAP];
T_KALMAN = [T_KALMAN Time_KALMAN];
T_MAP = [T_MAP Time_MAP];
end
figure,
plot(1:T,xtr(1,:),'r')
hold on
plot(1:T,x_est(1,:),'b')
hold on

plot(1:T,x_est_MAP(1,:),'g')
legend('True','Kalman','MAP')
title('Tracking for position')


figure,
plot(1:T,xtr(2,:),'r')
hold on
plot(1:T,x_est(2,:),'b')
hold on
plot(1:T,x_est_MAP(2,:),'g')
title('Tracking for velocity')
legend('True','Kalman','MAP')

figure,
plot(mm,Trmse_1,'r');
hold on;
plot(mm,Trmse_2,'b');
legend('TRMSE','TRMSE_{MAP}');
ylabel('TRMSE Vector');
xlabel('m');
legend('TRMSE Kalman','TRMSE MAP based Kalman')

figure,
plot(mm,T_KALMAN,'r')
hold on
plot(mm,T_MAP,'b')
xlabel('m')
ylabel('Elapsed time')
legend('Elapsed time KALMAN','Elapsed time MAP EKF')
