% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here

mu = [149.6987 144.5827 60.8693];
covar = [192.8401  120.5356 -204.9874;
         120.5356  128.8537 -162.8509;
        -204.9874 -162.8509  355.5739];
thre = 0.000001;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
[N,M,D] = size(I);
l = N*M;
pdf = zeros(l,1);
x = double(reshape(I, l, D));

for i=1:l
    pdf(i) = (x(i,:)-mu) * inv(covar) * (x(i,:)-mu)';
end
pdf = 1/sqrt(((2*pi)^D)*det(covar)) .* exp(-0.5 * pdf);
mask = reshape(pdf, N, M) > thre;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

CC = bwconncomp(mask);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
segI = false(N,M);
segI(CC.PixelIdxList{idx}) = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid; 

% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
