clear all
close all
%tic
Trmse_1=[];
Trmse_2=[];
Trmse_3=[];
time_kalman = [];
time_MAP = [];

x_est_MAP = [];
tic
for m = 2:2:2
    
%  for lambda = 0:0.02:0.2
delT = 1;             % sampling Time
F=[1 delT 0 0;0 1 0 0;0 0 1 delT;0 0 0 1];    

alpha = 1000;
mean_observation_outlier=[0 0 0 0];
lambda = 0;

L =2;  
Position_diff=[];
Position_diff_MAP=[];
Velocity_diff = [];
Velocity_diff_MAP = [];
%for l = 1:L
% m =6;

dt=1;
T = 1000;
nstates = 4;         % numbers of States
nmeas = 4;           % number of measurements 
Q= 0.5*eye(4);

P0 = Q; % Set initial error covariance
x0=[700;180;400;90];
xt= x0;
MAP_prob_vec=[];

outlier_store=[];
vt_store=[];
prob_diff=[];
xtr=[];
elapsed_time = zeros(1,m);
%tic
for k = 1:T
    
    xt(:,k) = F*xt+[mvnrnd([0 0 0 0],Q)]';
    xt = xt(:,k);
    
    
    n1=1;
    n2=2;

     r1=(n1*10^-4);                    % This Matrix is defined only once because it is constant for all
     r2=n2^2; % This Matrix is defined only once because it is constant for al   

    R=zeros(m,m);
    for i=1:2:m
        R(i,i)= r1;
    end
    
    for i=2:2:m
        R(i,i)= r2;
    end

    for i =1:m/2
    n=2*i-1;
    if rand<lambda
    ynoisy(n,k)=atan2(xt(3)-(350*(mod(i,2))),xt(1)-(i-1)*350)+mvnrnd(0,r1*alpha);
    else
    ynoisy(n,k)=atan2(xt(3)-(350*(mod(i,2))),xt(1)-(i-1)*350)+mvnrnd(0,r1);
    end
    end

    %event  = rand;
%     event_total = [event_total event];
    
    for i=1:m/2
    n=2*i;
    if rand<lambda
    ynoisy(n,k)=sqrt((xt(1)-(i-1)*350)^2+(xt(3)-(350*mod(i-1,2)))^2)+mvnrnd(0,n2^2*alpha);
    else
    ynoisy(n,k)=sqrt((xt(1)-(i-1)*350)^2+(xt(3)-(350*mod(i-1,2)))^2)+mvnrnd(0,n2^2);
    end
    end
   


    
             
    
    
 
xtr = [xtr xt];
end
[x_est,P] = KF_func(x0,P0,T,F,Q,m,R,ynoisy,nstates);

%MAP


[x_est_MAP] = MAP_KF_independent_2_dim(F,x0,Q,m,ynoisy(:,k),R,T);
%[x_est_MAP] = MAP_est(m,ynoisy(:,k),Q,x0,T,F,R);

%% Calculating Time averaged root mean square (TRMSE)
diff=xtr-x_est;
Position_diff=cat(3, Position_diff,[diff(1,:);diff(3,:)]);
Velocity_diff = cat(3,Velocity_diff,[diff(2,:);diff(4,:)]);
% % 
diff_MAP=xtr-x_est_MAP;
Position_diff_MAP=cat(3, Position_diff_MAP,[diff_MAP(1,:);diff_MAP(3,:)]);
Velocity_diff_MAP = cat(3,Velocity_diff_MAP,[diff_MAP(2,:);diff_MAP(4,:)]);

% %% Final TRMSE
TRMSE=1/T*sum(sqrt((1/L)*sum(sum(Position_diff.^2+Velocity_diff.^2),3)));
TRMSE_MAP=1/T*sum(sqrt((1/L)*sum(sum(Position_diff_MAP.^2+Velocity_diff_MAP.^2),3)));    
Trmse_1=[Trmse_1 TRMSE];
Trmse_2=[Trmse_2 TRMSE_MAP];
% % end
% 
t_kalman = toc;
time_kalman = [time_kalman t_kalman];
t_MAP = toc;
time_MAP = [time_MAP t_MAP];
% 
end
%toc
% end
% % % 
figure,
plot(1:T,xtr(1,:),'r')
hold on
plot(1:T,x_est(1,:),'b')
hold on

plot(1:T,x_est_MAP(1,:),'g')
legend('True','Kalman','MAP')
title('Tracking for position')
% 
% 
% figure,
% plot(1:T,xtr(2,:),'r')
% hold on
% plot(1:T,x_est(2,:),'b')
% hold on
% plot(1:T,x_est_MAP(2,:),'g')
% title('Tracking for velocity')
% legend('True','Kalman','MAP')

figure,
plot(2:2:4,Trmse_1,'r');
hold on;
plot(2:2:4,Trmse_2,'b');
ylabel('TRMSE Vector');
xlabel('m');
legend('TRMSE Kalman','TRMSE MAP based Kalman')

figure,
plot(2:2:4,time_kalman,'r');
hold on;
plot(2:2:4,time_MAP,'b');
ylabel('Elapsed time');
xlabel('m');
legend('Elapsed time Kalman','Elapsed time MAP');

% % toc