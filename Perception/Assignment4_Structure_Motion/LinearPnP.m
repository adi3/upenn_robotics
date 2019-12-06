function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

n = size(X, 1);
x = [x ones(n, 1)];
X = [X ones(n, 1)];
xc = (K\x')';   % A\b ~ inv(A)*b

A = zeros(3*n, 12);
z = zeros(1, 4);

for i = 1:n
    Xi = [X(i,:) z z; z X(i,:) z; z z X(i,:)];
    A(3*i-2:3*i, :) = Vec2Skew(xc(i,:)) * Xi;
end

[U,D,V] = svd(A);
P = reshape(V(:, end), 4, 3)';
P = P/P(end);

R = P(:,1:3);
t = P(:,4);

[U,D,V] = svd(R);
sign = det(U*V');

R = sign*U*V';
t = sign*t/D(1,1);
C = -R'*t;


