clear all
close all

Trmse_1=[];
Trmse_2=[];
Trmse_3=[];
elapsed_time_Kalman = [];
elapsed_time_MAP = [];
elapsed_time_OEKF = [];
lambda = 0.2;
mm =2:2:2;
T_KALMAN = [];
T_MAP = [];
T_OEKF = [];
for m = mm
delT = 1;             % sampling Time
F=[1 delT 0 0;0 1 0 0;0  0 1 delT;0 0 0 1];    
alpha = 1000;
mean_observation_outlier=[0 0 0 0];

L =5;  
T = 1000;
xtr_ch = zeros(m,L,T);
x_est_ch = zeros(m,L,T);
x_est_MAP_ch = zeros(m,L,T);
x_est_Paper_ch = zeros(m,L,T);
parfor l = 1:L
Position_diff=[];
Position_diff_MAP=[];
Velocity_diff = [];
Velocity_diff_MAP = []; 
Position_diff_OEKF = [];
Velocity_diff_OEKF = [];

disp(l)    
dt=1;
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

[xtr,ynoisy] = Data_Gen(F,Q,lambda,T,x0,m,alpha,R);
% tic
%EKF

[x_est] = EKF_func(m,F,x0,P0,T,Q,R,ynoisy,nstates);
% Time_KALMAN = toc;



%MAP estimation
% tStart = tic;
% x_aposteriori_previous_MAP=x0;
% P_aposteriori_previous_MAP=Q;
% P_apriori_current_MAP=F*P_aposteriori_previous_MAP*F'+ Q;
% x_apriori_current_MAP=F*x_aposteriori_previous_MAP;

[x_est_MAP] =ModifiedMAPbasedEKF(F,Q,x0,T,m,ynoisy,R);
% Time_MAP = toc(tStart); 


% tStart1 = tic;
[x_est_Paper] = moEKF1(F,Q,x0,T,m,ynoisy,R);
% Time_OEKF = toc(tStart1);
%% Calculating Time averaged root mean square (TRMSE)
% diff=xtr-x_est;
% Position_diff=cat(3, Position_diff,[diff(1,:);diff(3,:)]);
% Velocity_diff = cat(3,Velocity_diff,[diff(2,:);diff(4,:)]);
% 
% 
% diff_MAP=xtr-x_est_MAP;
% Position_diff_MAP=cat(3, Position_diff_MAP,[diff_MAP(1,:);diff_MAP(3,:)]);
% Velocity_diff_MAP = cat(3,Velocity_diff_MAP,[diff_MAP(2,:);diff_MAP(4,:)]);
% 
% diff_OEKF = xtr-x_est_Paper;
% Position_diff_OEKF = cat(3, Position_diff_OEKF,[diff_OEKF(1,:);diff_OEKF(3,:)]);
% Velocity_diff_OEKF = cat(3,Velocity_diff_OEKF,[diff_OEKF(2,:);diff_OEKF(4,:)]);
% 
end
% %% Final TRMSE
% TRMSE=1/T*sum(sqrt((1/L)*sum(sum(Position_diff.^2+Velocity_diff.^2),3)));
% TRMSE_MAP=1/T*sum(sqrt((1/L)*sum(sum(Position_diff_MAP.^2+Velocity_diff_MAP.^2),3)));
% TRMSE_OEKF = 1/T*sum(sqrt((1/L)*sum(sum(Position_diff_OEKF.^2+Velocity_diff_OEKF.^2),3)));
% Trmse_1=[Trmse_1 TRMSE];
% Trmse_2=[Trmse_2 TRMSE_MAP];
% Trmse_3 = [Trmse_3 TRMSE_OEKF];
% T_KALMAN = [T_KALMAN Time_KALMAN];
% T_MAP = [T_MAP Time_MAP];
% T_OEKF = [T_OEKF Time_OEKF];
end
% figure,
% plot(1:T,x(1,:),'r')
% hold on
% plot(1:T,x_est(1,:),'b')
% hold on
% plot(1:T,x_est_MAP(1,:),'g')
% hold on
% plot(1:T,x_est_Paper(1,:),'y')
% legend('True','Kalman','MAP','Modified EKF')
% title('Tracking for position')
% 
% 
% figure,
% plot(1:T,x(2,:),'r')
% hold on
% plot(1:T,x_est(2,:),'b')
% hold on
% plot(1:T,x_est_MAP(2,:),'g')
% hold on
% plot(1:T,x_est_Paper(2,:),'y')
% title('Tracking for velocity')
% legend('True','Kalman','MAP','Modified EKF')
% 
% figure,
% plot(mm,Trmse_1,'r');
% hold on;
% plot(mm,Trmse_2,'b');
% hold on
% plot(mm,Trmse_3,'g');
% ylabel('TRMSE Vector');
% xlabel('m');
% legend('TRMSE Kalman','TRMSE MAP based Kalman','TRMSE Modified EKF')
% 
% figure,
% plot(mm,T_KALMAN,'r')
% hold on
% plot(mm,T_MAP,'b')
% hold on
% plot(mm,T_OEKF,'g')
% xlabel('m')
% ylabel('Elapsed time')
% legend('Elapsed time KALMAN','Elapsed time MAP EKF','Elapsed time Modified EKF')
