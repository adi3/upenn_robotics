function [endeff] = computeMiniForwardKinematics(rads1,rads2)

L1 = 1;
L2 = 2;

alpha = pi + (rads1 + rads2)/2;
beta = (rads1 - rads2)/2;

phi = 3*pi/2 - alpha;
psi = alpha - rads1;

syms l;

eqn = l^2 - (2*L1*cos(psi))*l + (L1^2 -L2^2) == 0;
soln = eval(solve(eqn, l));

l = soln(soln > 0);
x = -l*sin(phi);
y = -l*cos(phi);
    
endeff = [x,y];