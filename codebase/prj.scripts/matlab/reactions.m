clear variables

syms N_1 N_2 N_3 N_4 real
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

R_1 = N_1*[ kx_1, cos(alpha) + kz_1*sin(alpha), -sin(alpha) + kz_1*cos(alpha)]';
R_2 = N_2*[ kx_2, cos(alpha) + kz_2*sin(alpha), -sin(alpha) + kz_2*cos(alpha)]';
R_3 = N_3*[ kx_3, ky_3, 1]';
R_4 = N_4*[ kx_4, ky_4, 1]';
F_c = [0, 0, -P]';


eq1 = R_1(1) + R_2(1) + R_3(1) + R_4(1) == F_c(1);
eq2 = R_1(2) + R_2(2) + R_3(2) + R_4(2) == F_c(2);
eq3 = R_1(3) + R_2(3) + R_3(3) + R_4(3) == F_c(3);

mom_R_1 = cross(r_1, R_1);
mom_R_2 = cross(r_2, R_2);
mom_R_3 = cross(r_3, R_3);
mom_R_4 = cross(r_4, R_4);
mom_F_c = cross(r_c, F_c);

eq4 = mom_R_1(1) + mom_R_2(1) + mom_R_3(1) + mom_R_4(1) == mom_F_c(1);
eq5 = mom_R_1(2) + mom_R_2(2) + mom_R_3(2) + mom_R_4(2) == mom_F_c(2);
eq6 = mom_R_1(3) + mom_R_2(3) + mom_R_3(3) + mom_R_4(3) == mom_F_c(3);

eq7 = N_1 > 0;
eq8 = N_2 > 0;
eq9 = N_3 > 0;
eq10 = N_4 > 0;
eq11 = P > 0;
eq12 = y_c > 0;
eq13 = z_c > 0;
eq14 = h > 0;
eq15 = l > 0;
eq16 = alpha >= 0;
eq17 = y_c < l;




equations = [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10, eq11, eq12, eq13, eq14, eq15, eq16, eq17];

%equations = subs(equations, alpha, 0);

syms kx real
syms kz real
syms ky real

equations = subs(equations, [kx_1, kx_2, kx_3, kx_4],[kx, kx, kx, kx]);
equations = subs(equations, [kz_1, kz_2], [kz, kz]);
equations = subs(equations, [ky_3, ky_4], [ky, ky]);

for i = 1:6
    pretty(simplify(equations(i)))    
end

syms N_u real
syms N_d real

equations = subs(equations, [N_1, N_2], [N_u, N_u]);
equations = subs(equations, [N_3, N_4], [N_d, N_d]);


for i = 1:6
    pretty(simplify(equations(i)))    
end

equations = subs(equations, alpha, 0);
% 
% for i = 1:6
%     pretty(simplify(equations(i)))    
% end

syms k real
syms N real

equations = subs(equations, [kx, ky, kz], [0,k,k]);
%equations = subs(equations, [N_u, N_d], [N, N]);

disp('ololo')
for i = 1:6
   pretty(simplify(equations(i)))    
end

solution = solve(equations,[N_u, N_d, k], 'ReturnConditions', true)
