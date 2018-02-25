clc
clear variables

syms N_1 N_2 N_3 N_4 real
syms Ft_1 Ft_2 Ft_3 Ft_4 real
syms Fnu_1 Fnu_2 Fnu_3 Fnu_4 real

syms alpha d h l H real

syms knu_1 knu_2 knu_3 knu_4 real
syms kt_1 kt_2 kt_3 kt_4 real

syms P real
syms y_c z_c real

%step points for legs
r_1 = [ d, l, H]';
r_2 = [-d, l, H]';
r_3 = [ d, h*tan(alpha), h]';
r_4 = [-d, h*tan(alpha), h]';

%center of gravity
r_c = [0, y_c, z_c]';

%orthogonal components of reactions
N_1_vec = N_1*[0, 0, 1]';
N_2_vec = N_2*[0, 0, 1]';
N_3_vec = N_3*[0, cos(alpha), -sin(alpha)]';
N_4_vec = N_4*[0, cos(alpha), -sin(alpha)]';

%tangent components of reactions (orthogonal to Ox axis)
Ft_1_vec = Ft_1*[0, 1, 0]';
Ft_2_vec = Ft_2*[0, 1, 0]';
Ft_3_vec = Ft_3*[0, sin(alpha), cos(alpha)]';
Ft_4_vec = Ft_4*[0, sin(alpha), cos(alpha)]';

%second tangent components of reactions (along Ox axis)
Fnu_1_vec = Fnu_1*[1,0,0]';
Fnu_2_vec = Fnu_2*[1,0,0]';
Fnu_3_vec = Fnu_3*[1,0,0]';
Fnu_4_vec = Fnu_4*[1,0,0]';

%gravity force
F_c = [0, 0, -P]';

%projections of all forces onto the coordinate axes
eq1 = N_1_vec(1) + N_2_vec(1) + N_3_vec(1) + N_4_vec(1) + Ft_1_vec(1) + Ft_2_vec(1) + Ft_3_vec(1) + Ft_4_vec(1) + F_c(1) + Fnu_1_vec(1) + Fnu_2_vec(1) + Fnu_3_vec(1) + Fnu_4_vec(1) == 0;
eq2 = N_1_vec(2) + N_2_vec(2) + N_3_vec(2) + N_4_vec(2) + Ft_1_vec(2) + Ft_2_vec(2) + Ft_3_vec(2) + Ft_4_vec(2) + F_c(2) + Fnu_1_vec(2) + Fnu_2_vec(2) + Fnu_3_vec(2) + Fnu_4_vec(2) == 0;
eq3 = N_1_vec(3) + N_2_vec(3) + N_3_vec(3) + N_4_vec(3) + Ft_1_vec(3) + Ft_2_vec(3) + Ft_3_vec(3) + Ft_4_vec(3) + F_c(3) + Fnu_1_vec(3) + Fnu_2_vec(3) + Fnu_3_vec(3) + Fnu_4_vec(3) == 0;

%calculating all force momentums
mom_N_1_vec = cross(r_1, N_1_vec);
mom_N_2_vec = cross(r_2, N_2_vec);
mom_N_3_vec = cross(r_3, N_3_vec);
mom_N_4_vec = cross(r_4, N_4_vec);
mom_Ft_1_vec = cross(r_1, Ft_1_vec);
mom_Ft_2_vec = cross(r_2, Ft_2_vec);
mom_Ft_3_vec = cross(r_3, Ft_3_vec);
mom_Ft_4_vec = cross(r_4, Ft_4_vec);
mom_F_c = cross(r_c, F_c);

mom_Fnu_1_vec = cross(r_1, Fnu_1_vec);
mom_Fnu_2_vec = cross(r_2, Fnu_2_vec);
mom_Fnu_3_vec = cross(r_3, Fnu_3_vec);
mom_Fnu_4_vec = cross(r_4, Fnu_4_vec);

%projection of all force momentums onto the coordinate axes
eq4 = mom_N_1_vec(1) + mom_N_2_vec(1) + mom_N_3_vec(1) + mom_N_4_vec(1) + ...
      mom_Ft_1_vec(1) + mom_Ft_2_vec(1) + mom_Ft_3_vec(1) + mom_Ft_4_vec(1) + mom_F_c(1) + ...
      mom_Fnu_1_vec(1) + mom_Fnu_2_vec(1) + mom_Fnu_3_vec(1) + mom_Fnu_4_vec(1)== 0;

eq5 = mom_N_1_vec(2) + mom_N_2_vec(2) + mom_N_3_vec(2) + mom_N_4_vec(2) + ...
      mom_Ft_1_vec(2) + mom_Ft_2_vec(2) + mom_Ft_3_vec(2) + mom_Ft_4_vec(2) + mom_F_c(2) + ...
      mom_Fnu_1_vec(2) + mom_Fnu_2_vec(2) + mom_Fnu_3_vec(2) + mom_Fnu_4_vec(2)== 0;

eq6 = mom_N_1_vec(3) + mom_N_2_vec(3) + mom_N_3_vec(3) + mom_N_4_vec(3) + ...
      mom_Ft_1_vec(3) + mom_Ft_2_vec(3) + mom_Ft_3_vec(3) + mom_Ft_4_vec(3) + mom_F_c(3) + ...
      mom_Fnu_1_vec(3) + mom_Fnu_2_vec(3) + mom_Fnu_3_vec(3) + mom_Fnu_4_vec(3)== 0;

eq7 = N_1 > 0;
eq8 = N_2 > 0;
eq9 = N_3 > 0;
eq10 = N_4 > 0;
eq11 = P > 0;
%eq12 = y_c > 0;
eq13 = z_c > 0;
eq14 = h > 0;
eq15 = y_c > l;
eq16 = alpha > 0;
eq17 = alpha < pi/2;
eq18 = H > 0;

equations = [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10, eq11, eq13, eq14, eq15, eq16, eq17, eq18];


%switching into the right angle cliff case
equations = subs(equations, alpha, 0);

disp('Initial equations:')
for i = 1:6
    pretty(simplify(expand(equations(i))));
end
disp('---------------------------------------')


%applying assumption of the Coulomb friction F_i = k*N_i
equations = subs(equations, [Ft_1, Ft_2, Ft_3, Ft_4], [kt_1*N_1, kt_2*N_2, kt_3*N_3, kt_4*N_4]);
equations = subs(equations, [Fnu_1, Fnu_2, Fnu_3, Fnu_4], [knu_1*N_1,knu_2*N_2,knu_3*N_3,knu_4*N_4]);

disp('Subs: F_i = k_i*N_i')
for i = 1:6
    pretty(simplify(equations(i)))    
end
disp('---------------------------------------')

syms knu real
syms kt_u real
syms kt_d real


%assumption that the configuration is symmetric in terms of friction
equations = subs(equations, [knu_1, knu_2, knu_3, knu_4],[knu, -knu, knu, -knu]);
equations = subs(equations, [kt_1, kt_2], [kt_u, kt_u]);
equations = subs(equations, [kt_3, kt_4], [kt_d, kt_d]);


disp('Subs: knu, kt_u, kt_d')
for i = 1:6
    pretty(simplify(equations(i)))    
end
disp('---------------------------------------')

syms N_u real
syms N_d real

%assumption that left and right sides of the robot loaded equally - gallop
%gait
equations = subs(equations, [N_1, N_2], [N_u, N_u]);
equations = subs(equations, [N_3, N_4], [N_d, N_d]);

disp('N_1 = N_2 = N_u; N_3 = N_4 = N_d')
for i = 1:6
    pretty(simplify(equations(i)));   
end

syms kt real

%assumption that front and rear legs loaded equally
equations = subs(equations, [kt_d, kt_u], [kt, -kt]);
%equations = subs(equations, P, 10);
%equations = subs(equations, [l, y_c, h], [1, 0.5, 1]);

disp('Final system:')
for i = 1:6
   pretty(simplify(equations(i)))    
end

solution = solve(equations, [N_u, N_d, kt], 'ReturnConditions', true)
solution.conditions
solution.parameters
