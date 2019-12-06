% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 

% % the number of grids for 1 meter.
resol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
origin = param.origin; 

% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

[M,N] = size(ranges);

for i = 1:N
    x = pose(1,i);
    y = pose(2,i);
    theta = pose(3,i);
    for j = 1:M
        d = ranges(j,i);
        alpha = scanAngles(j);
        x_occ = d * cos(theta + alpha) + x;
        y_occ = -d * sin(theta + alpha) + y;
        occ = ceil(resol * [x_occ y_occ]') + origin;
        robot = ceil(resol * [x y]') + origin;
        [freex, freey] = bresenham(robot(1), robot(2), occ(1), occ(2));
        
        free = sub2ind(param.size, freey, freex);
        myMap(occ(2), occ(1)) =  myMap(occ(2), occ(1)) + lo_occ;
        myMap(free) = myMap(free) - lo_free;
    end
end

myMap(myMap < lo_min) = lo_min;
myMap(myMap > lo_max) = lo_max;

end

