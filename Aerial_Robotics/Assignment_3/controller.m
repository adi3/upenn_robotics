function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters
%   -- mass, I, invI, gravity, arm_length, minF, maxF
     
%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

m = params.mass;
g = params.gravity;

% Thrust
kp_lin = 20;
kd_lin = 50;

t_cap = des_state.vel/norm(des_state.vel);
n_cap = des_state.acc/norm(des_state.acc);
b_cap = cross(t_cap, n_cap);
p_delta = des_state.pos - state.pos;

ep = dot(p_delta, n_cap)*n_cap + dot(p_delta, b_cap)*b_cap;
if (any(isnan(b_cap)))
    ep = des_state.pos - state.pos;
end
ev =  des_state.vel - state.vel;

r_ddot_des = des_state.acc + kd_lin * ev + kp_lin * ep;
F = m*g + m*r_ddot_des(3);

% Moment
kp_rot = 10;
kd_rot = 1;

psi_des = des_state.yaw;
phi_des = (1/g)*(r_ddot_des(1)*sin(psi_des) - r_ddot_des(2)*cos(psi_des));
theta_des = (1/g)*(r_ddot_des(1)*cos(psi_des) + r_ddot_des(2)*sin(psi_des));

rot_des = [phi_des; theta_des; psi_des];
omega_des = [0; 0; des_state.yawdot];

M = kp_rot * (rot_des - state.rot) + kd_rot * (omega_des - state.omega);

% =================== Your code ends here ===================

end
