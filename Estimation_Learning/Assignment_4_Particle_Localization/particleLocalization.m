% Robotics: Estimation and Learning
% WEEK 4
%
% Complete this function following the instruction.
function myPose = particleLocalization(ranges, scanAngles, map, param)
% myPose is a 3 by N matrix representing (x, y, theta) at each timestep.

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters
%
% % the number of grids for 1 meter.
resol = param.resol;
% % the origin of the map in pixels
origin = param.origin;

% The initial pose is given
myPose(:,1) = param.init_pose;

% You should put the given initial pose into pose for j=1, ignoring the j=1 ranges.
% The pose(:,j) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 200;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
% P = repmat(myPose(:,1), [1, M]);

sig = 0.05;
occ_threshold = 0.5;

for j = 2:N
  
    % Make all future generations replications of
    % last know best fit pose. Resampling not needed
    % since we fill up M particles on each run
    P = repmat(myPose(:,j-1), [1, M]) + randn(3,M) * sig;
    W = zeros(1, M);

    for i = 1:M
        x = P(1,i);
        y = P(2,i);
        theta = P(3,i);

        occ = zeros(2, size(ranges,1));
        for r = 1:size(ranges,1)
            alpha = scanAngles(r);
            x_hit = ranges(r,j) .* cos(theta + alpha) + x;
            y_hit = -ranges(r,j) .* sin(theta + alpha) + y;
            cell_hit = ceil(resol .* [x_hit y_hit]') + origin;  
         
            occ(:,r) = [cell_hit(1) cell_hit(2)]';
        end
        
        % On a 2D grid, X-axis -> columns, Y-axis -> rows
        % Valid cells are those inside map boundary
        valid = (occ(1,:) > 0) & (occ(1,:) < size(map,2)) ...
              & (occ(2,:) > 0) & (occ(2,:) < size(map,1));
        occ = occ(:,valid);

        idx = sub2ind(size(map),occ(2,:),occ(1,:));
        W(i) = sum(map(idx) > occ_threshold);
     end

     % Choose best fit pose
     [~, idx] = max(W);
     myPose(:,j) = P(:,idx);
     
     j*100/N
end

end
