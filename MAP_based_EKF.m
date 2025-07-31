n=5;             % number of states
R_t  = 10^2;     % covariance of measurement  
dt   = 1;
q1   = 1;
q2   = 1.75*(10^-4);
alpha  = 300;
lambda = 0.2;
T    = 100;
x_sym = sym('x', [5, 1]);
Q= [q1*dt^3/3 q1*dt^2/2 0 0 0;...
    q1*dt^2/2 q1*dt 0 0 0;...
    0 0 q1*dt^3/3 q1*dt^2/2 0;...
    0 0 q1*dt^2/2 q1*dt 0;...
    0 0 0 0 q2];                     % This Matrix is defined only once because it is constant for all% time points
P = Q;                               % initial state covraiance
R = diag([50^2 , 1.6*10^-3]);
f= [[1 sin(x_sym(5)*dt)/x_sym(5) 0 (cos(x_sym(5)*dt)-1)/x_sym(5) 0 ; 0 cos(x_sym(5)*dt) 0 -sin(x_sym(5)*dt) 0; 0 (1-cos(x_sym(5)*dt))/x_sym(5) 1 sin(x_sym(5)*dt)/x_sym(5) 0; 0 sin(x_sym(5)*dt) 0 cos(x_sym(5)*dt) 0;0 0 0 0 1] * x_sym] + [mvnrnd([0 0 0 0 0],Q)]' ;  % nonlinear state equations
N  = T/dt;                                                   % total dynamic steps
L  = 10;
u1  = zeros(5,N);
u2  = zeros(2,N);
u1(:,1) = [-10000;10;5000;-5;-0.053];                             % initial state vector
u2(:,1) = [sqrt(u1(1,1)^2 + u1(1,3)^2) ; abs(atan2(u1(1,3),u1(1,1)))];        % initial output vector     
x = u1(:,1);                                                      % initial set state          

xV = zeros(n,N);                                             % estmate        
sV = zeros(n,N);                                             % actual
zV = zeros(2,N);
z  = zeros(2,1);
error = zeros(N,1);
%alpha_arr  = 50:50:1000; 
%lambda_arr = 0:0.05:0.9;
% TRSME_Ext_Kal_Var_alpha  = zeros(length(alpha_arr),1);
% TRSME_Ext_Kal_Var_lambda = zeros(length(lambda_arr),1);

 alpha  = 300;
 lambda = 0.2;
 h=[sqrt(x_sym(1)^2 + x_sym(3)^2) ; abs(atan2(x_sym(3),x_sym(1)))] + [(1 - lambda).*mvnrnd([0 0],R) + lambda*mvnrnd([0 0],alpha.*R)]';                               % measurement equation
 pi = [0.6,0.2,0.2];
 sigma_1 = diag([10^12,0]);
 sigma_2 = diag([0,10*3.14]);
 F       = jacobian(f,x_sym);
 H       = jacobian(h,x_sym);
 s       = u1(:,1);
 
 for k=1:N
        temp = 0;
        for j=1:L 
            x = s;
            x1 = s(1);
            x2 = s(2);
            x3 = s(3);
            x4 = s(4);
            x5 = s(5);
            z = subs(h);                               % measurments
            sV(:,k)= s;                             % save actual state    
            zV(:,k)  = z;                           % save measurment
            [x, P] = ekf(f,h,x,P,h,z,Q,R_t,pi,sigma_1,sigma_2,F,H);          % ekf 
            xV(:,k) = x;                            % save estimate
            s(1) = x(1);
            s(2) = x(2);
            s(3) = x(3);
            s(4) = x(4);
            s(5) = x(5);
            x1 = s(1);
            x2 = s(2);
            x3 = s(3);
            x4 = s(4);
            x5 = s(5);
            s = subs(f);                               % update process 
            
            % Compute Error
            temp = temp + sqrt((xV(1,k)' - sV(1,k)').^2 + (xV(3,k)' - sV(3,k)').^2) ;
            disp(temp);
        end
        error(k,1) = sqrt(temp/L);
 end;
 TRSME = sum(error)/T;