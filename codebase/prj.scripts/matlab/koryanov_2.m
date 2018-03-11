clc;
clear variables;

syms N psi kappa k

hi = 2*sin(psi)/(kappa*2*k*N)

N_d = (hi*N + 1 - sqrt(hi^2*N^2 + 1))/(2*hi)
N_u = (hi*N - 1 + sqrt(hi^2*N^2 + 1))/(2*hi)

k_t = (kappa*2*k*N)/(2*N_u*cos(psi)) + sin(psi)/cos(psi)
