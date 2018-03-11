clc
clear variables
syms N_u N_d k P h y_c l H real


% ((H-h) + sqrt((H-h^2 - 4*y_c^2 + 4*l*y_c)))/(2*y_c)

eqs = [((H-h) + sqrt(((H-h)^2 - 4*y_c^2 + 4*l*y_c)))/(2*y_c) < 0;
        l < 0;
        H-h > 0;
        y_c > 0];
    
    
solve(eqs)


