function Thrusters = GenerateCornerThrusters( u, l, w, h)
    R_x = @(theta)([1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)]);
    R_y = @(theta)([cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)]);
    R_z = @(theta)([cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1]);

    Thrusters{1} = [[l/2, w/2, -h/2]';R_z(u(1))*R_y(u(9))*R_x(u(17))*[0,-1,0]']';
    Thrusters{2} = [[-l/2, w/2, -h/2]';R_z(u(2))*R_x(-u(10))*R_y(u(18))*[1,0,0]']';
    Thrusters{3} = [[-l/2, -w/2, -h/2]';R_z(u(3))*R_y(-u(11))*R_x(u(19))*[0,1,0]']';
    Thrusters{4} = [[l/2, -w/2, -h/2]';R_z(u(4))*R_x(u(12))*R_y(u(20))*[-1,0,0]']';

    Thrusters{5} = [[l/2, w/2, h/2]';R_z(u(5))*R_y(-u(13))*R_x(u(21))*[0,-1,0]']';
    Thrusters{6} = [[-l/2, w/2, h/2]';R_z(u(6))*R_x(u(14))*R_y(u(22))*[1,0,0]']';
    Thrusters{7} = [[-l/2, -w/2, h/2]';R_z(u(7))*R_y(u(15))*R_x(u(23))*[0,1,0]']';
    Thrusters{8} = [[l/2, -w/2, h/2]';R_z(u(8))*R_x(-u(16))*R_y(u(24))*[-1,0,0]']';
end