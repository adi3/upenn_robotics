
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  
  persistent ei t_last
  if isempty(ei)
      ei = 0;
      t_last = 0;
  end
  
  kp = 100;
  kd = 1;
  ki = 1000;
  
  dt = t - t_last;
  ei = ei + phi*dt;
  
  u = kp*phi + kd*phidot + ki*ei;
  t_last = t;
end

