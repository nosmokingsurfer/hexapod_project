clear variables

syms N_1 N_2 N_3 N_4 real
syms F_1 F_2 F_3 F_4 real
syms alpha d h l real

syms kx_1 kx_2 kx_3 kx_4 real
syms kz_1 kz_2 ky_3 ky_4 real

syms P real
syms y_c z_c real


r_1 = [ d, h*tan(alpha), h]';
r_2 = [-d, h*tan(alpha), h]';
r_3 = [  d, l, 0]';
r_4 = [ -d, l, 0]';
r_c = [0, y_c, z_c]';

N_1_vec = N_1*[0, cos(alpha), -sin(alpha)]';
N_2_vec = N_2*[0, cos(alpha), -sin(alpha)]';
N_3_vec = N_3*[0, 0, 1]';
N_4_vec = N_4*[0, 0, 1]';

F_1_vec = F_1*[0, sin(alpha), cos(alpha)]';
F_2_vec = F_2*[0, sin(alpha), cos(alpha)]';
F_3_vec = F_3*[0, -1, 0]';
F_4_vec = F_4*[0, -1, 0]';

F_c = [0, 0, -P]';


eq1 = N_1_vec(1) + N_2_vec(1) + N_3_vec(1) + N_4_vec(1) + F_1_vec(1) + F_2_vec(1) + F_3_vec(1) + F_4_vec(1) + F_c(1) == 0
eq2 = N_1_vec(2) + N_2_vec(2) + N_3_vec(2) + N_4_vec(2) + F_1_vec(2) + F_2_vec(2) + F_3_vec(2) + F_4_vec(2) + F_c(2) == 0
eq3 = N_1_vec(3) + N_2_vec(3) + N_3_vec(3) + N_4_vec(3) + F_1_vec(3) + F_2_vec(3) + F_3_vec(3) + F_4_vec(3) + F_c(3) == 0

mom_N_1_vec = cross(r_1, N_1_vec);
mom_N_2_vec = cross(r_2, N_2_vec);
mom_N_3_vec = cross(r_3, N_3_vec);
mom_N_4_vec = cross(r_4, N_4_vec);
mom_F_1_vec = cross(r_1, F_1_vec);
mom_F_2_vec = cross(r_2, F_2_vec);
mom_F_3_vec = cross(r_3, F_3_vec);
mom_F_4_vec = cross(r_4, F_4_vec);
mom_F_c = cross(r_c, F_c);

eq4 = mom_N_1_vec(1) + mom_N_2_vec(1) + mom_N_3_vec(1) + mom_N_4_vec(1) + ...
    mom_F_1_vec(1) + mom_F_2_vec(1) + mom_F_3_vec(1) + mom_F_4_vec(1) + mom_F_c(1) == 0

eq5 = mom_N_1_vec(2) + mom_N_2_vec(2) + mom_N_3_vec(2) + mom_N_4_vec(2) + ...
    mom_F_1_vec(2) + mom_F_2_vec(2) + mom_F_3_vec(2) + mom_F_4_vec(2) + mom_F_c(2) == 0

eq6 = mom_N_1_vec(3) + mom_N_2_vec(3) + mom_N_3_vec(3) + mom_N_4_vec(3) + ...
    mom_F_1_vec(3) + mom_F_2_vec(3) + mom_F_3_vec(3) + mom_F_4_vec(3) + mom_F_c(3) == 0


eq7 = N_1 > 0
eq8 = N_2 > 0
eq9 = N_3 > 0
eq10 = N_4 > 0
eq11 = P > 0
eq12 = y_c > 0
eq13 = z_c > 0
eq14 = h > 0
eq15 = l > 0
eq16 = alpha >= 0
eq17 = alpha < pi/2

equations = [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10, eq11, eq12, eq13, eq14, eq15];%, eq16, eq17];

disp('Initial equations:')
for i = 1:6
    pretty(simplify(equations(i)))    
end


equations = subs(equations, [F_1, F_2, F_3, F_4], [kz_1*N_1, kz_2*N_2, ky_3*N_3, ky_4*N_4]);


syms kx real
syms kz real
syms ky real

equations = subs(equations, [kx_1, kx_2, kx_3, kx_4],[kx, -kx, kx, -kx]);
equations = subs(equations, [kz_1, kz_2], [kz, kz]);
equations = subs(equations, [ky_3, ky_4], [ky, ky]);

for i = 1:6
    pretty(simplify(equations(i)))    
end

syms N_u real
syms N_d real

equations = subs(equations, [N_1, N_2], [N_u, N_u]);
equations = subs(equations, [N_3, N_4], [N_d, N_d]);

disp('N_1 = N_2 = N_u; N_3 = N_4 = N_d')
for i = 1:6
    pretty(simplify(equations(i)))    
end

syms k real

%equations = subs(equations, [ky, kz], [-k, k]);
%equations = subs(equations, P, 10);
%equations = subs(equations, [l, y_c, h], [1, 0.5, 1]);

disp('Final system:')
for i = 1:6
   pretty(simplify(equations(i)))    
end

solution = solve(equations, [N_u, N_d, k], 'ReturnConditions', true)
solution.conditions
solution.parameters
