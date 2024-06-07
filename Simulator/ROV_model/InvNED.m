function Tau_inv = InvNED()
    syms  x y z phi theta pzi real
    Tau = sym('Tau', [6 1]);
    assume(Tau, 'real')

    J_1 = [cos(pzi)*cos(theta), -sin(pzi)*cos(phi)+cos(pzi)*sin(theta)*sin(phi),   sin(pzi)*sin(phi)+cos(pzi)*cos(phi)*sin(theta) ;...
           sin(pzi)*cos(theta), cos(pzi)*cos(phi)+sin(phi)*sin(theta)*sin(pzi) , -cos(pzi)*sin(phi)+sin(theta)*sin(pzi)*cos(phi);...
              -sin(theta)     ,             cos(theta)*sin(phi)               ,               cos(theta)*cos(phi)]                ;

    J_2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
           0,      cos(phi)      ,      -sin(phi)     ;...
           0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    J = [J_1, zeros(3,3); zeros(3,3), J_2];

    Tau_inv = J\Tau;

    open_system('modelsimulator')

    matlabFunctionBlock('modelsimulator/PID/Tau_body/InvNED', Tau_inv(1), Tau_inv(2), Tau_inv(3), Tau_inv(4), Tau_inv(5), Tau_inv(6), ...
        'vars', [Tau' x y z phi theta pzi], ...
        'Outputs', {'x_dot','y_dot','z_dot','phi_dot','theta_dot','pzi_dot',})

end