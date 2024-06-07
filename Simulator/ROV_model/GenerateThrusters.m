function Thrusters = GenerateThrusters(pos_rot, u, l, w, h)
    Thrusters = cell(numel(u),1);
    for i = 1: length(u)
         Thruster = pos_rot{u(i)};
         Thrusters{i} = [Thruster(1)*l, Thruster(2)*w, Thruster(3)*h, Thruster(4:6)];
    end 

end