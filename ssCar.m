function dxdt = ssCar(x,u)
    b = 0.2;
    
    vel = 0.5*(u(1)+u(2));
    ang_vel = (u(1)-u(2))/b;
    
    dxdt = [vel*cos(x(3));
            vel*sin(x(3));
            ang_vel];
end