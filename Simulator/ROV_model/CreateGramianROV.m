clc
clear

I = diag([0.26, 0.23, 0.37]);
COG = [0,0,0.0121];

J_3 = @(alpha)([cos(alpha),-sin(alpha),0;sin(alpha),cos(alpha),0;0,0,1]);

Thrusters{1} = [J_3(0)*     [0.156 -0.111 -0.085]'; J_3(0)*     [1/sqrt(2),  1/sqrt(2), 0]']';
Thrusters{2} = [J_3(-5.05)* [0.156 -0.111 -0.085]'; J_3(-pi/2)* [1/sqrt(2),  1/sqrt(2), 0]']';
Thrusters{3} = [J_3(pi)*    [0.156  0.111 -0.085]'; J_3(pi)*    [1/sqrt(2), -1/sqrt(2), 0]']';
Thrusters{4} = [J_3(1.91)*  [0.156  0.111 -0.085]'; J_3(-pi/2)* [1/sqrt(2), -1/sqrt(2), 0]']';
 
Thrusters{5} = [J_3(0)*     [0.120 0.218 0]'; [0 0  1]']';
Thrusters{6} = [J_3(4.15)*  [0.120 0.218 0]'; [0 0 -1]']';
Thrusters{7} = [J_3(1.01)*  [0.120 0.218 0]'; [0 0 -1]']';
Thrusters{8} = [J_3(pi)*    [0.120 0.218 0]'; [0 0  1]']';

tic;
Gramian = Create_Gramian_ROV(13.5,I,COG,0.378, 0.575, 0.457,Thrusters,[1 1 0 0 0 0]', [0 0 0 0.2 0.3 0]', [0 0 0 0 0 0]', 1);
toc;


function Gramian = Create_Gramian_ROV(m,I,COG,l,w,h,Thrusters, Velocity, Position, V0, Reduced)

ROV = initializeModelMatrices(m,I,COG,l,w,h,Thrusters);
linsysROV = linearize_ROV(ROV, Velocity, Position, V0, Reduced);
Gramian = gram(linsysROV,'c');

end

% ROV.[M,C,D,G,J,T,A_nl,B_nl]
function ROV_parameters = initializeModelMatrices(m,I,COG,l,w,h,Thrusters)
g = 9.82;
rho = 1000;
dim = [l,w,h ];
Ap = [dim(2)*dim(3) dim(1)*dim(2) dim(3)*dim(1)]*1e6*0.7;
Buoyancy = m*g; %rho*g*0.0134;
Weight = m*g;
COB = [0, 0, 0];
deltaBG = norm(COB-COG);
Ma = -added_mass(dim, Ap);
[Ds,Dm] = damping_estimation(dim,Ap,I,-Ma,deltaBG);

syms u v w p q r x y z phi theta pzi real



% Mass Matrix
    % restructured added mass derivatives
    mass_derivativ = [  Ma(1,1),    Ma(1,2),    Ma(1,3),    Ma(1,4),    Ma(1,5),    Ma(1,6);
                        Ma(1,2),    Ma(2,2),    Ma(2,3),    Ma(2,4),    Ma(2,5),    Ma(2,6);
                        Ma(1,3),    Ma(2,3),    Ma(3,3),    Ma(3,4),    Ma(3,5),    Ma(3,6);   
                        Ma(1,4),    Ma(2,4),    Ma(3,4),    Ma(4,4),    Ma(4,5),    Ma(4,6);   
                        Ma(1,5)     Ma(2,5),    Ma(3,5),    Ma(4,5),    Ma(5,5),    Ma(5,6);
                        Ma(1,6),    Ma(2,6),    Ma(3,6),    Ma(4,6),    Ma(5,6),    Ma(6,6);];

    % offdiagonal mass terms
    odm = [ 0,          m*COG(3),   -m*COG(2); 
            -m*COG(3),  0,          m*COG(1);
            m*COG(2),   -m*COG(1),  0       ];

    ROV_parameters.M = [diag([m,m,m]), odm; -odm, I]-mass_derivativ;


% Coriolis Matrix
    crb12 = m*[(COG(2)*q+COG(3)*r), -COG(1)*q+w,        -COG(1)*r-v;
               -COG(2)*p-w,         COG(3)*r+COG(1)*p,  -COG(2)*r+u;
               -COG(3)*p+v,         -COG(3)*q-u,        COG(1)*p+COG(2)*q];
    crb22 = [0,                           -I(2,1)*q-I(1,3)*p+I(3,3)*r,    I(2,1)*r+I(1,2)*p-I(2,2)*q;
             I(2,1)*q+I(1,3)*p-I(3,3)*r,  0,                              -I(1,3)*r-I(1,2)*q+I(1,1)*p;
             -I(2,1)*r-I(1,2)*p+I(2,2)*q, I(1,3)*r+I(1,2)*q-I(1,1)*p,     0];
    Crb = [zeros(3), crb12;-crb12, crb22];

    a = -mass_derivativ*[u;v;w;p;q;r];

    ca12 = [ 0,  -a(3),  a(2);
            a(3),  0,  -a(1);
            -a(2),  a(1), 0];
    ca22 = [ 0,  -a(6),  a(5);
             a(6),  0,  -a(4);
            -a(5),  a(4), 0];
    Ca = [zeros(3), ca12; ca12, ca22];

    ROV_parameters.C = Ca+Crb;


% Damping Matrix
    ROV_parameters.D = diag([abs(u),abs(v),abs(w),abs(p),abs(q),abs(r)]*Dm)+Ds;


% Restoring Forces 
    ROV_parameters.G = [ ...
        (Weight-Buoyancy)*sin(theta);
        -(Weight-Buoyancy)*cos(theta)*sin(phi);
        -(Weight-Buoyancy)*cos(theta)*cos(phi);
        -(COG(2)*Weight-COB(2)*Buoyancy)*cos(theta)*cos(phi) + (COG(3)*Weight-COB(3)*Buoyancy)*cos(theta)*sin(phi);
         (COG(3)*Weight-COB(3)*Buoyancy)*sin(theta)          + (COG(1)*Weight-COB(1)*Buoyancy)*cos(theta)*cos(phi);
        -(COG(1)*Weight-COB(1)*Buoyancy)*cos(theta)*sin(phi) - (COG(2)*Weight-COB(2)*Buoyancy)*sin(theta)];


% Kinematic transformation matrix
    J_1 = [cos(pzi)*cos(theta), -sin(pzi)*cos(phi)+cos(pzi)*sin(theta)*sin(phi),   sin(pzi)*sin(phi)+cos(pzi)*cos(phi)*sin(theta) ;...
           sin(pzi)*cos(theta), cos(pzi)*cos(phi)+sin(phi)*sin(theta)*sin(pzi) , -cos(pzi)*sin(phi)+sin(theta)*sin(pzi)*cos(theta);...
              -sin(theta)     ,             cos(theta)*sin(phi)               ,               cos(theta)*cos(phi)]                ;

    J_2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
           0,      cos(phi)      ,      -sin(phi)     ;...
           0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    ROV_parameters.J = [J_1, zeros(3,3); zeros(3,3), J_2];

    V0 = sym('v0_', [6 1]);
    assume(V0, 'real')
    ROV_parameters.J_s = jacobian(ROV_parameters.J*V0, [x y z phi theta pzi]);


% Thruster config matrix    
    ROV_parameters.T = Thrust_config_matrix(Thrusters,COG, 0);

    
% State Space Model
    A_nl = [-ROV_parameters.M\(jacobian(ROV_parameters.C*[u;v;w;p;q;r],[u v w p q r]) ...
        +jacobian(ROV_parameters.D*[u;v;w;p;q;r],[u v w p q r])), ...
        -ROV_parameters.M\jacobian(ROV_parameters.G,[x y z phi theta pzi]); ROV_parameters.J, ROV_parameters.J_s];
    A_reduced = A_nl(1:11,1:11); A_reduced(:,7:9) = []; A_reduced(7:9,:) = [];

    B_nl = [(ROV_parameters.M)\ROV_parameters.T; zeros([6,length(ROV_parameters.T)])];
    B_reduced = B_nl(1:11,:); B_reduced(7:9,:) = [];
    
    ROV_parameters.A_nl = A_nl;
    ROV_parameters.Ar = A_reduced;
    ROV_parameters.B_nl = B_nl;
    ROV_parameters.Br = B_reduced;

end

function linear_model = linearize_ROV(ROV_parameters, Velocity, Position, V0, Reduced)
R = ROV_parameters;
syms u v w p q r x y z phi theta pzi v0_1 v0_2 v0_3 v0_4 v0_5 v0_6 real

if Reduced
    A = double(subs(R.Ar, ...
                        [u v w p q r x y z phi theta pzi v0_1 v0_2 v0_3 v0_4 v0_5 v0_6], ...
                        [Velocity' Position' V0']));
    B = R.Br;
    C = eye(8);
    D = zeros(8);
else
    A = double(subs(R.A_nl, ...
                            [u v w p q r x y z phi theta pzi v0_1 v0_2 v0_3 v0_4 v0_5 v0_6], ...
                            [Velocity' Position' V0']));
    B = R.B_nl;
    C = eye(12);
    D = zeros(12,8);
end

linear_model = ss(A,B,C,D);
disp(A)
disp(B)


end