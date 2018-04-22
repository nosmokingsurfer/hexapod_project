clc
clear variables
syms alpha beta gamma real
syms p1 p2 p3 real

J = [-sin(alpha)*(p1 + p2*cos(beta) + p3*cos(beta + gamma)), -cos(alpha)*(p2*sin(beta) + p3*sin(beta+gamma)), -p3*cos(alpha)*sin(beta + gamma);
      cos(alpha)*(p1 + p2*cos(beta) + p3*cos(beta+gamma)),  -sin(alpha)*(p2*sin(beta) + p3*sin(beta + gamma)), -p3*sin(alpha)*sin(beta + gamma);
      0, p2*cos(beta) + p3*cos(beta + gamma), p3*cos(beta + gamma)]