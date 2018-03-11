%TODO
syms N_u N_d k P h y_c l real
syms alpha real

eqs = [N_d*k == N_u*(cos(alpha) + k*sin(alpha));
       2*N_d + 2*N_u*k*cos(alpha) == P + 2*N_u*sin(alpha);
       2*N_u*h + P*y_c*cos(alpha) == 2*N_d*l*cos(alpha);]
%        P > 0;
%        y_c > 0;
%        l > 0;
%        h > 0;
%        k > 0];
   
solution =  solve(eqs, [N_u, N_d, k])%, 'ReturnConditions', true)
pretty(solution.N_u)
