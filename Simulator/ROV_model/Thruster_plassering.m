clc
clear
close all
m=13.5;
dim = [0.67 0.465 0.35];
COB = [0 0 -0.0206];
h = dim(3);
l = dim(1);
w = dim(2);

vertices = [-l/2 -w/2 -h/2; l/2 -w/2 -h/2; l/2 w/2 -h/2; -l/2 w/2 -h/2; 
            -l/2 -w/2 h/2; l/2 -w/2 h/2; l/2 w/2 h/2; -l/2 w/2 h/2]; 

edges = [1 2; 2 3; 3 4; 4 1; 
         5 6; 6 7; 7 8; 8 5; 
         1 5; 2 6; 3 7; 4 8]; 

figure(1);
hold on;
for i = 1:size(edges, 1)
    plot3(vertices(edges(i, :), 1), vertices(edges(i, :), 2), vertices(edges(i, :), 3), 'b-');
end

%% Plot center of mass
cox = COB(1);
coy = COB(2);
coz = COB(3);

plot3(cox, coy, coz, 'ro', 'MarkerSize', 10); 

quiver3(cox, coy, coz, 0,0,-0.25,'k', 'LineWidth', 2)
quiver3(cox, coy, coz, 0,-0.25,0,'k', 'LineWidth', 2)
quiver3(cox, coy, coz, 0.25,0,0,'k', 'LineWidth', 2)

plot_size = 0.5;
plot3(plot_size, 0, 0, '', 'MarkerSize', 10); 
plot3(0, plot_size, 0, '', 'MarkerSize', 10); 
plot3(0, 0, plot_size, '', 'MarkerSize', 10); 
plot3(-plot_size, 0, 0, '', 'MarkerSize', 10); 
plot3(0, -plot_size, 0, '', 'MarkerSize', 10); 
plot3(0, 0, -plot_size, '', 'MarkerSize', 10); 


%% Trusters
% Most common thruster configuration
% Thrusters{1} = [ 0.5*l,  0.5*w, 0.5*h, 0, 0, -1];
% Thrusters{2} = [-0.5*l,  0.5*w, 0.5*h, 0, 0, -1];
% Thrusters{3} = [ 0.5*l, -0.5*w, 0.5*h, 0, 0, -1];
% Thrusters{4} = [-0.5*l, -0.5*w, 0.5*h, 0, 0, -1];
% 
% Thrusters{5} = [ 0.5*l,  0.5*w, COB(3),  1, -1, 0];
% Thrusters{6} = [-0.5*l,  0.5*w, COB(3), -1, -1, 0];
% Thrusters{7} = [ 0.5*l, -0.5*w, COB(3),  1,  1, 0];
% Thrusters{8} = [-0.5*l, -0.5*w, COB(3), -1,  1, 0];

% Common thuster configuration, thrusters in corners
% Thrusters{1} = [0.5*l, 0.5*w, 0.5*h, 2.5, -2, -2.5];
% Thrusters{2} = [-0.5*l, 0.5*w, 0.5*h, 2.5, 2, -2.5];
% Thrusters{3} = [0.5*l, -0.5*w, 0.5*h, 2.5, 2, -2.5];
% Thrusters{4} = [-0.5*l, -0.5*w, 0.5*h, 2.5, -2, -2.5];
% 
% Thrusters{5} = [0.5*l, 0.5*w, -0.5*h, 2.5, -2, 2.5];
% Thrusters{6} = [-0.5*l, 0.5*w, -0.5*h, 2.5, 2, 2.5];
% Thrusters{7} = [0.5*l, -0.5*w, -0.5*h, 2.5, 2, 2.5];
% Thrusters{8} = [-0.5*l, -0.5*w, -0.5*h, 2.5, -2, 2.5];

% Blue ROV configuration
R_x = @(theta)([1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)]);
J_3 = @(alpha)([cos(alpha),-sin(alpha),0;sin(alpha),cos(alpha),0;0,0,1]);

Thrusters{1} = [J_3(0)*[0.156 -0.111 -0.085]'; J_3(0)*[1/sqrt(2), 1/sqrt(2), 0]']';
Thrusters{2} = [J_3(-5.05)*[0.156 -0.111 -0.085]'; J_3(-pi/2)*[1/sqrt(2), 1/sqrt(2), 0]']';
Thrusters{3} = [J_3(pi)*[0.156 0.111 -0.085]'; J_3(pi)*[1/sqrt(2), -1/sqrt(2), 0]']';
Thrusters{4} = [J_3(1.91)*[0.156 0.111 -0.085]'; J_3(-pi/2)*[1/sqrt(2), -1/sqrt(2), 0]']';

Thrusters{5} = [J_3(0)*[0.120 0.218 0]'; [0 0 1]']';
Thrusters{6} = [J_3(4.15)*[0.120 0.218 0]'; [0 0 -1]']';
Thrusters{7} = [J_3(1.01)*[0.120 0.218 0]'; [0 0 -1]']';
Thrusters{8} = [J_3(pi)*[0.120 0.218 0]'; [0 0 1]']';

T = Thrust_config_matrix(Thrusters, COB,1,0,1);

%% Setting up the plot

xticks(-2:0.25:2);
yticks(-2:0.25:2);
zticks(-2:0.25:2);

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D ROV Visualization');

grid on;
axis equal;
hold off;

