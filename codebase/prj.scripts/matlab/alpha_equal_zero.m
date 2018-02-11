syms N_u N_d k P h y_c l real

eqs = [N_u - N_d*k == 0;
       2*N_d + 2*N_u*k == P;
       2*N_u*h + P*y_c == 2*N_d*l;
       P > 0;
       y_c > 0;
       l > 0;
       h > 0;
       k > 0];
   
solution =  solve(eqs, [N_u, N_d, k], 'ReturnConditions', true)
pretty(solution.N_u)
