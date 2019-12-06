
function u = controller(params, t, x, xd)
  % x = current position
  % xd = current velocity

  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error

  % params can be initialized in the initParams function, which is called before the simulation starts
  
  % SOLUTION GOES HERE -------------
  kp = 32;
  kd = 10;
  e = params.traj(t) - x;
  edot = 0 - xd;
  u = kp*e + kd*edot;
  
end