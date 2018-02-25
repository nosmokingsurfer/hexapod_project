clc
clear variables
[X,Y] = meshgrid(0:1:100,0:1:100);

Z1 = 2*(Y.*(Y+X)).^0.5;
Z2 = -2*(Y.*(Y+X)).^0.5;

hold on
surf(X,Y,Z1)
surf(X,Y,Z2)
xlabel('l')
ylabel('y_c')
zlabel('h')
grid on
