
function u = controller(params, t, X)
  % You have full state feedback available
  
  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*x
  K = [-0.0010   -0.1638   -0.0013   -0.0145];
  u = -K * X;
end

