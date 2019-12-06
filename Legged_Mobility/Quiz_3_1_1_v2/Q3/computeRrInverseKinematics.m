function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;

L1 = 1;
L2 = 1;

eq1 = X == L1 * cos(theta1) + L2 * cos(theta1 + theta2);
eq2 = Y == L1 * sin(theta1) + L2 * sin(theta1 + theta2);

soln = solve([eq1, eq2], theta1, theta2);
val1 = eval(mod(soln.theta1, 2*pi));
val2 = eval(mod(soln.theta2, 2*pi));

i = val1 > 0 & val1 < pi/2;

rads1 = val1(i,1);
rads2 = val2(i,1);