% Initialize Model Matrices

function ROV_parameters = initializeModelMatrices(m, I, COB, dim, blueROVHeavy)
g = 9.82;
COG = [0, 0, 0];

if blueROVHeavy
    m = 13.5;
    Buoyancy = m*g;
    Weight = m*g;
    COB = [0 0 -0.01];
    dim = [0.457 0.575 0.378];
    I   = diag([0.26, 0.23, 0.37]);
    Ma  = diag([6.36, 7.121, 18.69, 0.1858, 0.1348, 0.2215]);
    Ds  = diag([13.7, 0, 33.0, 0, 0.8, 0]);                                                       
    Dm  = diag([141, 217, 190, 1.192, 0.47, 1.5]);
else
    Buoyancy = m*g;
    Weight = m*g;
    deltaBG = norm(COB-COG);
    Ap = [dim(2)*dim(1) dim(3)*dim(2) dim(1)*dim(3)]*1e6*0.7;
    Ma = added_mass(dim, Ap);
    [Ds,Dm] = damping_estimation(dim,Ap,I,Ma,deltaBG,Weight);
end

syms u v w p q r x y z phi theta pzi real



ROV_parameters.COG = COG;



% Mass Matrix

    % offdiagonal mass terms
    odm = [ 0,          m*COG(3),   -m*COG(2); 
            -m*COG(3),  0,          m*COG(1);
            m*COG(2),   -m*COG(1),  0       ];

    ROV_parameters.M = [diag([m,m,m]), odm; -odm, I]+Ma;

% Coriolis and centripetal Matrix
    Crb = [m*skewMatrix([p q r]'), -m*skewMatrix([p q r]')*skewMatrix(COG);
                m*skewMatrix(COG)*skewMatrix([p q r]'),   -skewMatrix(I*[p q r]')];

    a = -Ma*[u;v;w;p;q;r];

    ca12 = [ 0,  -a(3),  a(2);
            a(3),  0,  -a(1);
            -a(2),  a(1), 0];
    ca22 = [ 0,  -a(6),  a(5);
             a(6),  0,  -a(4);
            -a(5),  a(4), 0];
    Ca = [zeros(3), ca12; ca12, ca22];

    ROV_parameters.C = Crb+Ca;


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
    ROV_parameters.J_s = jacobian(ROV_parameters.J*[u v w p q r]', [x y z phi theta pzi]);


% Thruster config matrix    
if blueROVHeavy
    J_3 = @(alpha)([cos(alpha),-sin(alpha),0;sin(alpha),cos(alpha),0;0,0,1]);

    Thrusters{1} = [J_3(0)*     [0.156 -0.111 -0.085]'; J_3(0)*     [1/sqrt(2),  1/sqrt(2), 0]']';
    Thrusters{2} = [J_3(-5.05)* [0.156 -0.111 -0.085]'; J_3(-pi/2)* [1/sqrt(2),  1/sqrt(2), 0]']';
    Thrusters{3} = [J_3(pi)*    [0.156  0.111 -0.085]'; J_3(pi)*    [1/sqrt(2), -1/sqrt(2), 0]']';
    Thrusters{4} = [J_3(1.91)*  [0.156  0.111 -0.085]'; J_3(-pi/2)* [1/sqrt(2), -1/sqrt(2), 0]']';
    
    Thrusters{5} = [[0.120 0.218 0]'; [0 0  1]']';
    Thrusters{6} = [[0.120 -0.218 0]'; [0 0 -1]']';
    Thrusters{7} = [[-0.120 0.218 0]'; [0 0 -1]']';
    Thrusters{8} = [[-0.120 -0.218 0]'; [0 0  1]']';
else
    Thrusters{1} = [ 0.2625,  0.175, COB(3), 1, -1, 0]'; %HHF
    Thrusters{2} = [-0.2625,  0.175, COB(3),-1, -1, 0]'; %HHB
    Thrusters{3} = [-0.2625, -0.175, COB(3),-1,  1, 0]'; %HVB
    Thrusters{4} = [ 0.2625, -0.175, COB(3), 1,  1, 0]'; %HVF

    Thrusters{5} = [ 0.23,  0.28, 0.147, 0, 0, 1]'; %VHF
    Thrusters{6} = [-0.23,  0.28, 0.147, 0, 0, 1]'; %VHB
    Thrusters{7} = [-0.23, -0.28, 0.147, 0, 0, 1]'; %VVB
    Thrusters{8} = [ 0.23, -0.28, 0.147, 0, 0, 1]'; %VVF
end

ROV_parameters.T = Thrust_config_matrix(Thrusters, COB, 0,0,0);


% Equations of motions expressed in NED

J = ROV_parameters.J;
J_dot = sym(zeros(size(J)));

% Loop over each element of J to compute its time derivative
for i = 1:size(J, 1)
    for j = 1:size(J, 2)
        % Compute the time derivative of J(i, j)
        J_dot(i, j) = diff(J(i, j), x) * u + ...
                      diff(J(i, j), y) * v + ...
                      diff(J(i, j), z) * w + ...
                      diff(J(i, j), phi) * p + ...
                      diff(J(i, j), theta) * q + ...
                      diff(J(i, j), pzi) * r;
    end
end
 
 ROV_parameters.M_ned = (J^-1)'*ROV_parameters.M*J^-1;
 ROV_parameters.C_ned = (J^-1)'*(ROV_parameters.C-ROV_parameters.M*(J^-1)*J_dot)*J^-1;
 ROV_parameters.D_ned = (J^-1)'*ROV_parameters.D*J^-1;
 ROV_parameters.G_ned = (J^-1)'*ROV_parameters.G;

    
% State Space Model
    A_nl = [-ROV_parameters.M\(jacobian(ROV_parameters.C*[u;v;w;p;q;r],[u v w p q r]) ...
        +jacobian(ROV_parameters.D*[u;v;w;p;q;r],[u v w p q r])), ...
        -ROV_parameters.M\jacobian(ROV_parameters.G,[x y z phi theta pzi]); ROV_parameters.J, ROV_parameters.J_s];
    A_reduced = A_nl(1:11,1:11); A_reduced(:,7:9) = []; A_reduced(7:9,:) = [];
    A_simple = A_nl(1:6,1:6);


    B_nl = [(ROV_parameters.M)\ROV_parameters.T; zeros([6,length(ROV_parameters.T)])];
    B_reg = [inv(ROV_parameters.M); zeros([6,6])];
    B_reduced = B_nl(1:11,:); B_reduced(7:9,:) = [];
    B_simple = B_nl(1:6,:);

    ROV_parameters.A_nl = A_nl;
    ROV_parameters.Ar = A_reduced;
    ROV_parameters.As = A_simple;
    ROV_parameters.B_nl = B_nl;
    ROV_parameters.Br = B_reduced;
    ROV_parameters.Bs = B_simple;
    ROV_parameters.Breg = B_reg;
end