%Good

clc
syms N_u N_d k P h y_c l real


% N_u == N_d kt
% 
% 2 N_d + 2 N_u kt == P
% 
% 2 N_u h + P y_c == 2 N_d l


eqs = [N_u - N_d*k == 0;
       2*N_d + 2*N_u*k == P;
       2*N_u*h + P*y_c == 2*N_d*l;
       P > 0;
       y_c > 0;
       h > 0;
       N_u > 0;
       N_d > 0;
       y_c < l;
       k > 0;];
   
solution =  solve(eqs, [N_u, N_d, k], 'ReturnConditions', true);
disp('N_u = ')
pretty(solution.N_u)
disp('N_d = ')
pretty(solution.N_d)
disp('k = ')
pretty(solution.k)