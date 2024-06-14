%% Validation of linearized model
% This script is dependent on the lin_val.slx file
clc
clear

m = 13.5;
dim = [0.457 0.575 0.378];
[I, COB] = InertiaDyadic(3,m,dim);

ROV = initializeModelMatrices(m, I, COB,dim,0);

v_dot = calcAcceleration_linval(ROV.M,ROV.C,ROV.D,ROV.G);
n_dot = calcNED_linval();

v_init = [1 0 0 0 0 0];
eta_init = [0 0 0 0 0 0];
F_diff = [20 0 0 0 0 0]';

linsysROV = linearize_ROV(ROV,v_init', eta_init', 0);
%%
syms u v w p q r x y z phi theta pzi real
Tau = sym('tau', [1 6]);
assume(Tau, 'real')

v_test = subs(v_dot,[u v w p q r x y z phi theta pzi], [v_init,eta_init]);

eqn = v_test == zeros(6,1);

solutions = solve(eqn,Tau);

F = [double(solutions.tau1); double(solutions.tau2); double(solutions.tau3); double(solutions.tau4); double(solutions.tau5); double(solutions.tau6)];

u = (ROV.T'/(ROV.T*ROV.T'))*F;
u_diff = (ROV.T'/(ROV.T*ROV.T'))*(F_diff);

function linear_model = linearize_ROV(ROV_parameters, Velocity, Position, Reduced)
R = ROV_parameters;
syms u v w p q r x y z phi theta pzi real
if Reduced
    A = double(subs(R.As, ...
                        [u v w p q r x y z phi theta pzi], ...
                        [Velocity' Position']));
    B = R.Bs;
    C = eye(6);
    D = zeros(6,8);
else
    A = double(subs(R.A_nl, ...
                            [u v w p q r x y z phi theta pzi], ...
                            [Velocity' Position']));
    B = R.B_nl;
    C = eye(12);
    D = zeros(12,8);
end

linear_model = ss(A,B,C,D);
disp(eig(A))
end

function [n_dot, J] = calcNED_linval()
    syms u v w p q r x y z phi theta pzi real
    J_1 = [cos(pzi)*cos(theta), -sin(pzi)*cos(phi)+cos(pzi)*sin(theta)*sin(phi),   sin(pzi)*sin(phi)+cos(pzi)*cos(phi)*sin(theta) ;...
           sin(pzi)*cos(theta), cos(pzi)*cos(phi)+sin(phi)*sin(theta)*sin(pzi) , -cos(pzi)*sin(phi)+sin(theta)*sin(pzi)*cos(phi);...
              -sin(theta)     ,             cos(theta)*sin(phi)               ,               cos(theta)*cos(phi)]                ;

    J_2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
           0,      cos(phi)      ,      -sin(phi)     ;...
           0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    J = [J_1, zeros(3,3); zeros(3,3), J_2];

    n_dot = J*[u, v, w, p, q, r]';

    open_system('lin_val')

    matlabFunctionBlock('lin_val/ROV/NED/calcNED', n_dot(1), n_dot(2), n_dot(3), n_dot(4), n_dot(5), n_dot(6), ...
        'vars', [u v w p q r x y z phi theta pzi], ...
        'Outputs', {'x_dot','y_dot','z_dot','phi_dot','theta_dot','pzi_dot',})

end

function v_dot = calcAcceleration_linval(M, C, D, G)
    syms u v w p q r x y z phi theta pzi real
    Tau = sym('tau', [1 6]);
    assume(Tau, 'real')
    v_dot = M\(-C*[u, v, w, p, q, r]'-D*[u, v, w, p, q, r]' - G + Tau');
    open_system('lin_val')
    
    matlabFunctionBlock('lin_val/ROV/ROVSubsystem/ROV', v_dot(1), v_dot(2), v_dot(3), v_dot(4), v_dot(5), v_dot(6), ...
        'vars', [u v w p q r x y z phi theta pzi Tau], ...
        'Outputs', {'u_dot','v_dot','w_dot','p_dot','q_dot','r_dot',})
    
end