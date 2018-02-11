clear all

syms N_u N_d k P l h H y_c real

equations = [2*N_u + 2*k*N_d == P;
             k*N_u == N_d;
             2*N_d*h + P*y_c == 2*N_u*(l+H*k);
             H > 0;
             P > 0;
             h > 0;
             y_c > l;
             N_u > 0;
             N_d > 0];
         
solution = solve(equations, [N_u, N_d, k], 'ReturnConditions', true)
