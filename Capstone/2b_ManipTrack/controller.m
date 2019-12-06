
function u = controller(params, t, X)
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  
  l = params.l;
  th1 = X(1);
  th2 = X(2);
  dth1 = X(3);
  dth2 = X(4);
  
  p = l*[cos(th1); sin(th1)] + l*[cos(th1+th2); sin(th1+th2)];
  e = params.traj(t) - p;
  edot = 0 - [dth1; dth2];
  
  J = [-l*(sin(th1) + sin(th1+th2)), -l*sin(th1+th2);
        l*(cos(th1) + cos(th1+th2)),  l*cos(th1+th2)];
      
  kp = 2000;
  kd = 10;
  u = kp*J'*e + kd*edot;
  
end

