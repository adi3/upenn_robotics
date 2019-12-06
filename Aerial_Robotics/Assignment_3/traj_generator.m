function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 order coeffs alphas

%{
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%}

%% Fill in your code here
%
order = 8;
coeffs = [1 1 1 1 1 1 1 1;
          0 1 2 3 4 5 6 7;
          0 0 2 6 12 20 30 42;
          0 0 0 6 24 60 120 210;
          0 0 0 0 24 120 360 840;
          0 0 0 0 0 120 720 2520;
          0 0 0 0 0 0 720 5040];

if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
          
    n = size(waypoints, 2) - 1;
    A = zeros(order*n, order*n);
    b = zeros(order*n, 3);
    n_eq = 0;
    
    % Constraint 19.1
    % t = S(i-1) ==> All equation variables will become 0
    % Only constant will remain non-zero
    for v = 1:n
       j = order*(v-1) + 1;
       A(n_eq+v, j) = 1;
       b(n_eq+v, :) = waypoints0(:, v)';
    end
    n_eq = n;
    
    % Constraint 19.2
    % t = S(i-1) + T(i) ==> All equation variables will become 1
    for v = 1:n
       j = order * (v-1);
       A(n_eq+v, j+1:j+order) = 1;
       b(n_eq+v, :) = waypoints0(:, v+1)';
    end
    n_eq = 2*n;
    
    % Constraint 20.1
    % Setting 1-, 2- and 3-order derivatives to 0 at start point
    for v=1:3
        j = v + 1;
        A(n_eq+v, j) = coeffs(v+1, j);
    end
    n_eq = 2*n + 3;
    
    % Constraint 20.2
    % Setting 1-, 2- and 3-order derivatives to 0 at end point
    for v=1:3
        j = order-1;
        A(n_eq+v, end-j:end) = coeffs(v+1, :);
    end
    n_eq = 2*n + 6;
    
    % Constraint 21
    % Setting 1- to 6-order derivatives for p(i) and p(i+1) equal at each waypoint
    for v = 1:n-1
        i = n_eq + 6*(v-1);
        j = order*(v-1);
        for k = 1:6
             % t = S(i-1) + T(i) ==> All equation variables will become 1
            A(i+k, j+1:j+order) = coeffs(k+1,:);
            % t = S(i-1) ==> All equation variables will become 0
            A(i+k, j+order+k+1) = -coeffs(k+1, k+1);
        end
    end
    
    alphas = [A\b(:,1) A\b(:,2) A\b(:,3)]';
else
    
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1) - 1;
    
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3, 1);
        desired_state.acc = zeros(3, 1);
    else
        range = (t_index - 1)*order + 1 : t_index*order;
        % Equation 18: f = (t - S(i-1))/T(i)
        scale = (t - traj_time(t_index))/d0(t_index);
        bases = zeros(order, 3);
        for k = 1:3
          for i = 1:order
            factor = (d0(t_index))^(k-1);
            bases(i, k) = (coeffs(k, i) * scale^(i-k))/factor;
          end
        end
    
        desired_state.pos = alphas(:, range) * bases(:, 1);
        desired_state.vel = alphas(:, range) * bases(:, 2);
        desired_state.acc = alphas(:, range) * bases(:, 3); 
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

end

