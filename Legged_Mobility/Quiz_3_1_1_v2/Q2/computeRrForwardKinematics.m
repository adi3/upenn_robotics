function [elbow,endeff] = computeRrForwardKinematics(rads1,rads2)
%%GIVEN THE ANGLES OF THE MOTORS, return an array of arrays for the
%%position of the elbow [x1,y1], and endeffector [x2,y2]
L1 = 1;
L2 = 1;

elbow_x = L1 * cos(rads1);
elbow_y = L1 * sin(rads1);

endeff_x = elbow_x + L2 * cos(rads1 + rads2);
endeff_y = elbow_y + L2 * sin(rads1 + rads2);

elbow = [elbow_x, elbow_y];
endeff = [endeff_x, endeff_y];
