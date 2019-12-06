function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
A = zeros(8, 9);

for i = 1:size(video_pts, 1)
    p = video_pts(i, :);
    p_ = logo_pts(i, :);
    A(2*i - 1, :) = [-p(1) -p(2) -1 0 0 0 p(1)*p_(1) p(2)*p_(1) p_(1)];
    A(2*i, :) = [0 0 0 -p(1) -p(2) -1 p(1)*p_(2) p(2)*p_(2) p_(2)];
end

[U, S, V] = svd(A);
h = V(:, end);
H = reshape(h, 3, 3)';

end

