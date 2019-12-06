function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% y-data
kv_y = 10;
kp_y = 40;

y = state.pos(1);
y_dot = state.vel(1);
y_des = des_state.pos(1);
y_dot_des = des_state.vel(1);
y_ddot_des = des_state.acc(1);

% z-data
kv_z = 50;
kp_z = 50;

z = state.pos(2);
z_dot = state.vel(2);
z_des = des_state.pos(2);
z_dot_des = des_state.vel(2);
z_ddot_des = des_state.acc(2);

% phi-data
kv_phi = 20;
kp_phi = 100;

phi = state.rot(1);
phi_dot = state.omega(1);
y_ddot = -(params.gravity*phi);
y_tdot_des = 0;   % assume y-des jerk is 0
phi_c_ddot = 0;   % approximate y-snap as 0

u1 = params.mass * (params.gravity + z_ddot_des + kv_z*(z_dot_des - z_dot) + kp_z*(z_des - z));

phi_c = (-1/params.gravity) * (y_ddot_des + kv_y*(y_dot_des - y_dot) + kp_y*(y_des - y));

phi_c_dot = (-1/params.gravity) * (y_tdot_des + kv_y*(y_ddot_des - y_ddot) + kp_y*(y_dot_des - y_dot));

u2 = params.Ixx * (phi_c_ddot + kv_phi*(phi_c_dot - phi_dot) + kp_phi*(phi_c - phi));

% FILL IN YOUR CODE HERE

end

