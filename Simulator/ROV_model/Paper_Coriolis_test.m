% clc
% clear

syms u v w p q r Xu_dot Yv_dot Zw_dot Kp_dot Mq_dot Nr_dot m I_x I_y I_z x_G y_G z_G
assume([u v w p q r Xu_dot Yv_dot Zw_dot Kp_dot Mq_dot Nr_dot m I_x I_y I_z x_G y_G z_G],'real')

 I = diag([I_x I_y I_z]);
 COG = [0, 0, 0];
% 
% % Added mass
 Ma = diag([Xu_dot Yv_dot Zw_dot Kp_dot Mq_dot Nr_dot]);
% 
% Mass rigid body
odm = [ 0,          m*COG(3),   -m*COG(2); 
            -m*COG(3),  0,          m*COG(1);
            m*COG(2),   -m*COG(1),  0       ];

Mrb = [diag([m,m,m]), odm; -odm, I];

% Total mass
M = Mrb + Ma;
% disp(M)

% Coriolis rigid body
crb12 = m*[(COG(2)*q+COG(3)*r), -COG(1)*q+w,        -COG(1)*r-v;
           -COG(2)*p-w,         COG(3)*r+COG(1)*p,  -COG(2)*r+u;
           -COG(3)*p+v,         -COG(3)*q-u,        COG(1)*p+COG(2)*q];
crb22 = [0,                           -I(2,1)*q-I(1,3)*p+I(3,3)*r,    I(2,1)*r+I(1,2)*p-I(2,2)*q;
         I(2,1)*q+I(1,3)*p-I(3,3)*r,  0,                              -I(1,3)*r-I(1,2)*q+I(1,1)*p;
         -I(2,1)*r-I(1,2)*p+I(2,2)*q, I(1,3)*r+I(1,2)*q-I(1,1)*p,     0];
Crb = [zeros(3), crb12;-crb12, crb22];


% Coriolis added
a = -Ma*[u;v;w;p;q;r];
ca12 = [ 0,  -a(3),  a(2);
            a(3),  0,  -a(1);
            -a(2),  a(1), 0];
ca22 = [ 0,  -a(6),  a(5);
             a(6),  0,  -a(4);
            -a(5),  a(4), 0];
Ca = [zeros(3), ca12; ca12, ca22];

% Total coriolis
% C = Crb + Ca;
% disp(C)
% 
% v_dot = M\(-C*[u, v, w, p, q, r]');
% 
% disp(v_dot)

Ca_test = [zeros(3,3), -skewMatrix(Ma(1:3,1:3)*[u v w]');
            -skewMatrix(Ma(1:3,1:3)*[u v w]'), -skewMatrix(Ma(4:6,4:6)*[p q r]')];
    
Ca_paper = [skewMatrix(Ma(1:3,1:3)*[p q r]'),                 -skewMatrix(Ma(1:3,1:3)*[p q r]')*skewMatrix(COG);
      skewMatrix(COG)*skewMatrix(Ma(1:3,1:3)*[p q r]'), -skewMatrix(Ma(4:6,4:6)*[p q r]')                ];

% disp(Ca)
% disp(Ca_test)
% disp(Ca_paper)
% disp(Ca_test*[u;v;w;p;q;r])
% disp(Ca_paper*[u;v;w;p;q;r])
%disp(Crb*[u;v;w;p;q;r])