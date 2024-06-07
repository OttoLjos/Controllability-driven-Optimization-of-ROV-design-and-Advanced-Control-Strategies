function Plot_ROV(l,w,h, COB, Thrusters)
COG = COB;

vertices = [-l/2 -w/2 -h/2; l/2 -w/2 -h/2; l/2 w/2 -h/2; -l/2 w/2 -h/2; 
            -l/2 -w/2 h/2; l/2 -w/2 h/2; l/2 w/2 h/2; -l/2 w/2 h/2]; 


edges = [1 2; 2 3; 3 4; 4 1; 
         5 6; 6 7; 7 8; 8 5; 
         1 5; 2 6; 3 7; 4 8]; 

hold on;
for i = 1:size(edges, 1)
    plot3(vertices(edges(i, :), 1), vertices(edges(i, :), 2), vertices(edges(i, :), 3), 'b-');
end

cox = COG(1);
coy = COG(2);
coz = COG(3);
plot3(cox, coy, coz, 'ro', 'MarkerSize', 5, 'LineWidth',2); % 'ro' means red circle

plot3(1, 0, 0, '', 'MarkerSize', 10); 
plot3(0, 1, 0, '', 'MarkerSize', 10); 
plot3(0, 0, 1, '', 'MarkerSize', 10); 
plot3(-1, 0, 0, '', 'MarkerSize', 10); 
plot3(0, -1, 0, '', 'MarkerSize', 10); 
plot3(0, 0, -1, '', 'MarkerSize', 10); 

Thrust_config_matrix(Thrusters, COB, 1,0,1);

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

end