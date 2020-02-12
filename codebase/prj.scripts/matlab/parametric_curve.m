clear variables;
clc;

syms t real
syms x0 x1 x2 x3 double
syms y0 y1 y2 y3 double




x = symfun( x0 + x1*t + x2*t^2 + x3*t^3, t);
dx = symfun(diff(x),t);
ddx = symfun(diff(x,2),t);

y = symfun( y0 + y1*t + y2*t^2 + y3*t^3, t);
dy = symfun(diff(y),t);
ddy = symfun(diff(y,2),t);

%R = \frac{(x'^2 + y'2)^{\frac 3 2}} {|x' y'' - y' x''|}

R = (dx^2 + dy^2)^(3/2)/(abs(dx*ddy - dy*ddx));


dR = symfun(diff(R),t);

t0 = [0,1];
dR = subs(dR, [x0,x1,x2,x3], [1,1,1,1]);
dR = subs(dR, [y0,y1,y2,y3], [1,2,3,4]);

t = 0:0.1:1;
plot(t,dR(t))