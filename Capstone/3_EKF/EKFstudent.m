
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data
  % you are provided a vector of timestamps (of length T),
  % and a 3xT matrix of observations, z.

  % Student completes this
  
  xhat = zeros(2,length(t));
  P = eye(2);
  Q = diag([0.1 0.1]);
  R = diag([0.01 0.01 0.01]);
  
  for i = 2:length(t)
      dt = t(i) - t(i-1);
      A = [1 dt; 0 1];
      xhat(:,i) = A*xhat(:,i-1);
      P = A*P*A' + Q;
      
      phi = xhat(1,i);
      phidot = xhat(2,i);
      
      % (pi/180) multiplier appears when chain rule is applied to
      % differentiate cos(phi*pi/180) where phi is in degrees.
      H = [cosd(phi)*(pi/180) -sind(phi)*(pi/180) 0; 0 0 1]'; 
      K = P*H'/(H*P*H' + R);
      
      h = [sind(phi) cosd(phi) phidot]';
      xhat(:,i) = xhat(:,i) + K*(z(:,i) - h);
      P = (eye(2) - K*H)*P;     
  end
  
end
