function [Xo,Po] = Extended_KF(f,g,Q,R,Z,Xi,Pi)
N_state = size(Xi, 1);    
[Xp, ~] = f(Xi);%1
[~, fy] = f(Xp);%2
[gXp, H] = g(Xp);%3
Pp = fy * Pi * fy.' + Q;%4
K = Pp * H' / (H * Pp * H.' + R);%5
    
Xo = Xp + K * (Z - gXp);%6
I = eye(N_state, N_state);
Po = (I - K * H) * Pp;%7
    