function [n_dot, J] = calcNED()
    syms u v w p q r x y z phi theta pzi real
    J_1 = [cos(pzi)*cos(theta), -sin(pzi)*cos(phi)+cos(pzi)*sin(theta)*sin(phi),   sin(pzi)*sin(phi)+cos(pzi)*cos(phi)*sin(theta) ;...
           sin(pzi)*cos(theta), cos(pzi)*cos(phi)+sin(phi)*sin(theta)*sin(pzi) , -cos(pzi)*sin(phi)+sin(theta)*sin(pzi)*cos(phi);...
              -sin(theta)     ,             cos(theta)*sin(phi)               ,               cos(theta)*cos(phi)]                ;

    J_2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
           0,      cos(phi)      ,      -sin(phi)     ;...
           0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    J = [J_1, zeros(3,3); zeros(3,3), J_2];

    n_dot = J*[u, v, w, p, q, r]';

    open_system('modelsimulator')

    matlabFunctionBlock('modelsimulator/ROV/NED/calcNED', n_dot(1), n_dot(2), n_dot(3), n_dot(4), n_dot(5), n_dot(6), ...
        'vars', [u v w p q r x y z phi theta pzi], ...
        'Outputs', {'x_dot','y_dot','z_dot','phi_dot','theta_dot','pzi_dot',})

end