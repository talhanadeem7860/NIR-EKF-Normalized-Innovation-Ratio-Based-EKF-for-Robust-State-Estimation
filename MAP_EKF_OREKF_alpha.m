clear all
close all

% Trmse_1=[];
% Trmse_2=zeros(10,4);
% Trmse_3=zeros(10,4);



% Trmse_1=[];
% Trmse_2=[];
% Trmse_3=[];
elapsed_time_Kalman = [];
elapsed_time_MAP = [];
elapsed_time_OEKF = [];
lambdaa = [0 0.3 0.5 0.6 0.7];
m =2;
alp =200:200:800;
Trmse_2=zeros(length(alp),length(lambdaa));
Trmse_3=zeros(length(alp),length(lambdaa));

T_KALMAN = [];
T_MAP = [];
T_OEKF = [];
for a = 1:length(lambdaa)
    lambda = lambdaa(a);
for ind = 1:length(alp)
    alpha = alp(ind);
delT = 1;             % sampling Time
F=[1 delT 0 0;0 1 0 0;0  0 1 delT;0 0 0 1];    
% alpha = 1000;
mean_observation_outlier=[0 0 0 0];

L =20;  

Position_diff=[];
Position_diff_MAP=[];
Velocity_diff = [];
Velocity_diff_MAP = []; 
Position_diff_OEKF = [];
Velocity_diff_OEKF = [];
for l = 1:L
disp(l)    
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



tStart1 = tic;
[x_est_Paper] = moEKF1(F,Q,x0,T,m,ynoisy,R);
Time_OEKF = toc(tStart1);
%% Calculating Time averaged root mean square (TRMSE)
diff=xtr-x_est;
Position_diff=cat(3, Position_diff,[diff(1,:);diff(3,:)]);
Velocity_diff = cat(3,Velocity_diff,[diff(2,:);diff(4,:)]);


diff_MAP=xtr-x_est_MAP;
Position_diff_MAP=cat(3, Position_diff_MAP,[diff_MAP(1,:);diff_MAP(3,:)]);
Velocity_diff_MAP = cat(3,Velocity_diff_MAP,[diff_MAP(2,:);diff_MAP(4,:)]);

diff_OEKF = xtr-x_est_Paper;
Position_diff_OEKF = cat(3, Position_diff_OEKF,[diff_OEKF(1,:);diff_OEKF(3,:)]);
Velocity_diff_OEKF = cat(3,Velocity_diff_OEKF,[diff_OEKF(2,:);diff_OEKF(4,:)]);

end
% %% Final TRMSE
% TRMSE=1/T*sum(sqrt((1/L)*sum(sum(Position_diff.^2+Velocity_diff.^2),3)));
TRMSE_MAP=1/T*sum(sqrt((1/L)*sum(sum(Position_diff_MAP.^2+Velocity_diff_MAP.^2),3)));
TRMSE_OEKF = 1/T*sum(sqrt((1/L)*sum(sum(Position_diff_OEKF.^2+Velocity_diff_OEKF.^2),3)));

Trmse_2(ind,a) = TRMSE_MAP;
Trmse_3(ind,a) = TRMSE_OEKF;

% Trmse_1=[Trmse_1 TRMSE];
% Trmse_2=[Trmse_2 TRMSE_MAP];
% Trmse_3 = [Trmse_3 TRMSE_OEKF];
% T_KALMAN = [T_KALMAN Time_KALMAN];
% T_MAP = [T_MAP Time_MAP];
% T_OEKF = [T_OEKF Time_OEKF];
end
end
% figure,
% plot(1:T,xtr(1,:),'r')
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
% plot(1:T,xtr(2,:),'r')
% hold on
% plot(1:T,x_est(2,:),'b')
% hold on
% plot(1:T,x_est_MAP(2,:),'g')
% hold on
% plot(1:T,x_est_Paper(2,:),'y')
% title('Tracking for velocity')
% legend('True','Kalman','MAP','Modified EKF')

figure,

% boxplot(Trmse_2,'colors','r',lambdaa)
% hold on
% boxplot(Trmse_3,'colors','b',lambdaa)
% plot(alp,Trmse_2(:,1),'go-', linewidth=2, markersize=6);
% hold on
% plot(alp,Trmse_3(:,1),'g*-', linewidth=2, markersize=6);
% 
% plot(alp,Trmse_2(:,2),'bo-', linewidth=2, markersize=6);
% plot(alp,Trmse_3(:,2),'b*-', linewidth=2, markersize=6);
% 
% plot(alp,Trmse_2(:,3),'ro-', linewidth=2, markersize=6);
% plot(alp,Trmse_3(:,3),'r*-', linewidth=2, markersize=6);
% 
% plot(alp,Trmse_2(:,4),'yo-', linewidth=2, markersize=6);
% plot(alp,Trmse_3(:,4),'y*-', linewidth=2, markersize=6);




data = {Trmse_2,Trmse_3};
boxplotGroup(data,'PrimaryLabels',{'alpha = 200' 'alpha = 400' 'alpha = 600' 'alpha = 800' 'alpha = 1000' 'alpha = 200' 'alpha = 400' 'alpha = 600' 'alpha = 800' 'alpha = 1000'},'SecondaryLabels',{'MAP','MOD'},'GroupLabelType','Vertical')
% boxplot([Trmse_2;Trmse_3],lambdaa,'colors','rb')
% hold on
% boxplot(Trmse_3,lambdaa,'colors','b')

% legend(findobj(gca,'Tag','Box'),'TRMSE_{MAP}','TRMSE_{MOD}')
% legend(findobj(gca,'Tag','Box'),'TRMSE_{MOD}')
ylabel('TRMSE Vector');
% xlabel('lambda');
title('m = 2, L = 40, alpha = 1000')
% boxes = findobj(gca, 'Tag', 'Box');
% legend(boxes([end 1]), 'TRMSE_{MAP}', 'TRMSE_{MOD}')

% legend('lambda_{MAP} = 0.0','lambda_{MOD} = 0.0','lambda_{MAP} = 0.3','lambda_{MOD} = 0.3','lambda_{MAP} = 0.5','lambda_{MOD} = 0.5','lambda_{MAP} = 0.7','lambda_{MOD} = 0.7')



% plot(alp,Trmse_1,'r');
% hold on;
% plot(alp,Trmse_2,'b');
% hold on
% plot(alp,Trmse_3,'g');
% ylabel('TRMSE Vector');
% xlabel('alpha');
% legend('TRMSE MAP based Kalman','TRMSE Modified EKF')
% legend('TRMSE Kalman','TRMSE MAP based Kalman','TRMSE Modified EKF')

% figure,
% % plot(alp,T_KALMAN,'r')
% % hold on
% plot(alp,T_MAP,'b')
% hold on
% plot(alp,T_OEKF,'g')
% xlabel('alpha')
% ylabel('Elapsed time')
% % legend('Elapsed time KALMAN','Elapsed time MAP EKF','Elapsed time Modified EKF')
% legend('Elapsed time MAP EKF','Elapsed time Modified EKF')
