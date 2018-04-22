% clc;
% clear variables;
% [X,Y]=meshgrid(0:0.05:4,1:0.05:3);
% Z = real(-0.5*X + (0.25*X.^2 + Y -1).^0.5);
% [C,h] = contour(X,Y,Z,40);
% 
% 
% clabel(C)
% hold on;
% axis equal;
% grid on;
% x = 0:0.05:4;
% plot(x,1-0.25*x.^2)
% plot(x,2 + x)



clc;
clear variables;
[X,Y]=meshgrid(-4:0.05:4,-4:0.05:4);
Z = real(0.5*X +(0.25*X.^2 + Y -1).^0.5);
Z(Z>1) = -1;
Z(Z<0) = -1;
[C,h] = contour(X,Y,Z,20);
clabel(C)
hold on;
axis equal;
grid on;
x = -4:0.05:4;
plot(x,1-0.25*x.^2)
plot(x,2 - x)

% [L,Y] = meshgrid(-10:0.5:0,-10:0.5:10);
% Z = L./Y;
% [C,h] = contour(L,Y,Z,10);
% clable(C)
% mesh(Z)


% [H,l] = meshgrid(0:0.025:1.5, -1.5:0.025:0);
% 
% for y = -1.0:0.05:1.0
%     figure()
%    Z = (H - (H^2-4*y.^2+4*l*y).^0.5)/(2.*y);
%    Z(imag(Z)>0) = 0;
%    Z = real(Z);
%    Z(Z>1)=0;
%    Z(Z<0) = 0;
% %    [C,h] = contour(H,l,Z,10);
% %    clabel(C);
%     mesh(Z)
%    xlabel('H-h');
%    ylabel('l');
%    title(y)
% end


