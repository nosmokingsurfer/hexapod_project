clc
clear variables
syms N_u N_d k P h y_c l H real


% N_d + N_u kt == 0
% 
% 2 N_u == P + 2 N_d kt
% 
% 2 N_d h + P y_c + 2 H N_u kt == 2 N_u l

eqs = [N_d - N_u*k == 0;
       2*N_u + 2*N_d*k == P;
       2*N_d*h + P*y_c == 2*N_u*l + 2*H*N_u*k];

s1 = solve(eqs(1), N_u);

eqs2 = subs(eqs(2), N_u, s1);

s2 = solve (eqs2, N_d);

s1 = subs(s1, N_d, s2);

eqs3 = subs(eqs(3), [N_u, N_d],[s1,s2]);

s_k = solve(eqs3, k);

s_N_d = subs(s1, s_k)
s_N_u = subs(s2, s_k)

pretty(simplify(s_k(1)))
pretty(simplify(s_k(2)))
   
