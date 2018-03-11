%GOOD
clc
clear variables
syms N_u N_d k P h y_c l H real


% N_d - N_u kt == 0
% 
% 2 N_u == P - 2 N_d kt
% 
% 2 N_d h + P y_c - 2 H N_u kt == 2 N_u l

eqs = [N_d - N_u*k == 0;
       2*N_u + 2*N_d*k == P;
       2*N_d*h + P*y_c == 2*N_u*l + 2*H*N_u*k;
       P > 0;       
       h > 0;
       H > h;
       N_u > 0;
       N_d > 0;
       y_c > l;
       l < 0;
       k > 0;];


solution =  solve(eqs, [N_u, N_d, k], 'ReturnConditions', true);
disp('N_u = ')
pretty(simplify(solution.N_u))
disp('N_d = ')
pretty(simplify(solution.N_d))
disp('k = ')
pretty(solution.k)


eqs_k1 = [solution.k(1) > 0;
        P > 0;
        y_c == 0;
        h > 0;
        H > h;
        N_u > 0;
        N_d > 0;
        y_c > l;
        l < 0;];
   
   
   s_k1 = solve(eqs_k1, [y_c])
