function ode_example()
    X0 = [1,0];
    tspan = [0,10];
    [t,X] = ode45(@shosc, tspan, X0);
    
    clf
    hold all
    plot(t, X(:,1))
    plot(t, X(:,2))
    hold off
end

function Xdot = shosc(t, X)
    x = X(1);
    xdot = X(2);
    Xdot = [xdot; -x];
end