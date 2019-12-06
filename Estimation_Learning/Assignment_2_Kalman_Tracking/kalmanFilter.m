function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covariances, etc. here:

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0]';
        param.P = 10 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    
    % Change of notation from Probablistic Robotics (Thrun)
    % A = A
    % C = C
    % Sigma = P
    % R = sigM
    % Q = R
    
    dt = t - previous_t; % should be ~0.033s
    A = [1  0  dt 0;
         0  1  0  dt;
         0  0  1  0;
         0  0  0  1];
    C = [1  0  0  0;
         0  1  0  0];
     
    sigM = [0.01  0     0.1  0;
            0     0.01  0    0.1;
            0.1   0     1    0;
            0     0.1   0    1];
    R = 0.01 * eye(2);
    
    z = [x y]';
    P = A * param.P * A' + sigM;
    
    K = P * C' * inv(R + C * P * C');
    state = A * state + K*(z - C * A * state);
    param.P = P - K * C * P;
    
    t = 0.33;   % 330ms
    predictx = state(1) + state(3)*t;
    predicty = state(2) + state(4)*t;
    
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    %vx = (x - state(1)) / (t - previous_t);
    %vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    %predictx = x + vx * 0.330;
    %predicty = y + vy * 0.330;
    % State is a four dimensional element
    %state = [x, y, vx, vy];
end
