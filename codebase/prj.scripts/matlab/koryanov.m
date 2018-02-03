clear variables

syms N_1 N_2 N_3 N_4 real
syms d z_1 z_2 real
syms k1_t k2_t k3_t k4_t real
syms k1_z k2_z k3_z k4_z real

syms psi real
syms P real
syms y_c z_c real


r_1 = [-d, 0, z_1]';
r_2 = [ d, 0, z_1]';
r_3 = [-d, 0, z_2]';
r_4 = [ d, 0, z_2]';
r_c = [0, y_c, z_c]';

R_1 = N_1*[-cos(psi) + k1_t*sin(psi), sin(psi) + k1_t*cos(psi), k1_z]';
R_2 = N_2*[ cos(psi) - k2_t*sin(psi), sin(psi) + k2_t*cos(psi), k2_z]';
R_3 = N_3*[-cos(psi) + k3_t*sin(psi), sin(psi) + k3_t*cos(psi), k3_z]';
R_4 = N_4*[ cos(psi) - k4_t*sin(psi), sin(psi) + k4_t*cos(psi), k4_z]';
F_c = [0, 0, -P]';


eq1 = R_1(1) + R_2(1) + R_3(1) + R_4(1) + F_c(1);
eq2 = R_1(2) + R_2(2) + R_3(2) + R_4(2) + F_c(2);
eq3 = R_1(3) + R_2(3) + R_3(3) + R_4(3) + F_c(3);

mom_R_1 = cross(r_1, R_1);
mom_R_2 = cross(r_2, R_2);
mom_R_3 = cross(r_3, R_3);
mom_R_4 = cross(r_4, R_4);
mom_F_c = cross(r_c, F_c);

eq4 = mom_R_1(1) + mom_R_2(1) + mom_R_3(1) + mom_R_4(1) + mom_F_c(1);
eq5 = mom_R_1(2) + mom_R_2(2) + mom_R_3(2) + mom_R_4(2) + mom_F_c(2);
eq6 = mom_R_1(3) + mom_R_2(3) + mom_R_3(3) + mom_R_4(3) + mom_F_c(3);

equations = [eq1, eq2, eq3, eq4, eq5, eq6];

equations = expand(equations);

equations = simplify(equations);

pretty(equations)


syms N_d N_u real
syms k_t_d k_t_u real
syms k_z_d k_z_u real


equations = subs(equations, [N_1, N_2], [N_d, N_d]);
equations = subs(equations, [N_3, N_4], [N_u, N_u]);
equations = subs(equations, [k1_t, k2_t], [k_t_d, k_t_d]);
equations = subs(equations, [k3_t, k4_t], [k_t_u, k_t_u]);
equations = subs(equations, [k1_z, k2_z], [k_z_d, k_z_d]);
equations = subs(equations, [k3_z, k4_z], [k_z_u, k_z_u]);

equations = simplify(equations);
pretty(equations);

syms k_z k_t real

equations = subs(equations, [k_z_d, k_z_u], [k_z, k_z]);
equations = subs(equations, [k_t_d, k_t_u], [k_t, -k_t]);
equations = simplify(equations);


pretty(equations)
s = solve(equations, [N_d, N_u, k_z])