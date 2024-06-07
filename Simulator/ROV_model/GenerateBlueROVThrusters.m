function Thrusters = GenerateBlueROVThrusters( u, l, w, h, COB)
    Thrusters{1} = [l/2, w/2, COB(3), cos(u(1)), -sin(u(1)),0];
    Thrusters{2} = [-l/2, w/2, COB(3), -cos(u(1)), -sin(u(1)),0];
    Thrusters{3} = [-l/2, -w/2, COB(3), -cos(u(1)), sin(u(1)),0];
    Thrusters{4} = [l/2, -w/2, COB(3), cos(u(1)), sin(u(1)),0];

    Thrusters{5} = [l/2, w/2, h/2, 0, -cos(u(2)),sin(u(2))];
    Thrusters{6} = [-l/2, w/2, h/2, 0, -cos(u(2)),sin(u(2))];
    Thrusters{7} = [-l/2, -w/2, h/2, 0, cos(u(2)),sin(u(2))];
    Thrusters{8} = [l/2, -w/2, h/2, 0, cos(u(2)),sin(u(2))];
end