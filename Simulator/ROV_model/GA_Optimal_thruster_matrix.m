%% Thruster Placements and orientations
clc
clear
close all
%%
m = 13.5;
dim = [0.457 0.575 0.378];
[I, COB] = InertiaDyadic(3,m,dim);

h = dim(3);
l = dim(1);
w = dim(2);

%Rest_mom = norm(COB)*m*9.81*sin(pi/2);

%%
NumPositions = 5;
NumSteps = 46;
Num_trusters = 8;

PopSize = 10000;
MaxGen = 1000000;

pos_rot_total = GeneratePosRot(NumPositions,NumSteps);
num_pos_rot_total = length(pos_rot_total);
%%
lb = ones(1, Num_trusters);
ub = num_pos_rot_total*ones(1, Num_trusters);
rng default

options = optimoptions('ga',...
                       'PopulationSize', PopSize, ...
                       'MaxGenerations', MaxGen,...
                       'CrossoverFraction', 0.7, ...
                       'PlotFcn','gaplotbestf');
[optimSol, fval] = ga(@(u)CostFunction(l, w, h, COB, pos_rot_total, u), Num_trusters, [], [], [], [], lb, ub, [], 1:Num_trusters, options);
%%
fig = figure(2);
Thrusters = GenerateThrusters(pos_rot_total, optimSol, l, w, h);
Plot_ROV(l,w,h, COB, Thrusters) 


T = Thrust_config_matrix(Thrusters, COB, 0,0,0);
Best_Cost = CostFunction(l,w,h, COB,pos_rot_total, optimSol);

disp('Cost of best result from GA:')
disp(Best_Cost)

%% Cost of manual options

Thrusters_man{1} = [0.5*l, 0.5*w, 0.5*h, 2.5, -2, -2.5];
Thrusters_man{2} = [-0.5*l, 0.5*w, 0.5*h, 2.5, 2, -2.5];
Thrusters_man{3} = [0.5*l, -0.5*w, 0.5*h, 2.5, 2, -2.5];
Thrusters_man{4} = [-0.5*l, -0.5*w, 0.5*h, 2.5, -2, -2.5];

Thrusters_man{5} = [0.5*l, 0.5*w, -0.5*h, 2.5, -2, 2.5];
Thrusters_man{6} = [-0.5*l, 0.5*w, -0.5*h, 2.5, 2, 2.5];
Thrusters_man{7} = [0.5*l, -0.5*w, -0.5*h, 2.5, 2, 2.5];
Thrusters_man{8} = [-0.5*l, -0.5*w, -0.5*h, 2.5, -2, 2.5];
% fig_man = figure(3);
% Plot_ROV(l,w,h, COB, Thrusters_man) 

Manual_T_1 = Thrust_config_matrix(Thrusters_man, COB, 0,0,0);
Manual_Cost_1 = Cost(Thrusters_man, COB);
disp('Cost of corner thrusters at 45x51x51 degrees:')
disp(Manual_Cost_1)

Thrusters_man{1} = [ 0.5*l,  0.5*w, 0.5*h, 0, 0, -1];
Thrusters_man{2} = [-0.5*l,  0.5*w, 0.5*h, 0, 0, -1];
Thrusters_man{3} = [ 0.5*l, -0.5*w, 0.5*h, 0, 0, -1];
Thrusters_man{4} = [-0.5*l, -0.5*w, 0.5*h, 0, 0, -1];

Thrusters_man{5} = [ 0.5*l,  0.5*w, -COB(3),  1, -1, 0];
Thrusters_man{6} = [-0.5*l,  0.5*w, -COB(3), -1, -1, 0];
Thrusters_man{7} = [ 0.5*l, -0.5*w, -COB(3),  1,  1, 0];
Thrusters_man{8} = [-0.5*l, -0.5*w, -COB(3), -1,  1, 0];

% fig_man = figure(4);
% Plot_ROV(l,w,h, COB, Thrusters_man) 

Manual_T_2 = Thrust_config_matrix(Thrusters_man, COB, 0,0,0);
Manual_Cost_2 = Cost(Thrusters_man, COB);
disp('Cost of Blue ROV type placement:')
disp(Manual_Cost_2)


%% Saving the data
folderPath = 'C:\Users\Ottol\Documents\UIS\Master Thesis\Matlab_Master\Ottos_Matlab_eventyr\ROV_model\GA_Results\Optimal_matrix';  % Change this to your desired path
% Create the folder if it doesn't exist
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

% Define file names for the data and figure
dataFileName = fullfile(folderPath, 'ga_data.mat');
gafigureFileName = fullfile(folderPath, 'ga_figure.fig');
ROVfigureFileName = fullfile(folderPath, 'ROV_figure.fig');
save(dataFileName, 'Num_trusters','NumPositions', "NumSteps", ...
    'num_pos_rot_total', 'PopSize', 'MaxGen',...
    'optimSol','T', 'Best_Cost', 'Thrusters', 'Manual_T_1', 'Manual_T_2','Manual_Cost_1', 'Manual_Cost_2');  % List other variables as needed
saveas(1, gafigureFileName);
saveas(fig, ROVfigureFileName);



function cost = CostFunction(l, w, h, COB, pos_rot_total, u)
    Thrusters = GenerateThrusters(pos_rot_total, u, l, w, h);    
    cost = Cost(Thrusters, COB);  
end

function cost = Cost(Thrusters, COB)    
    T = Thrust_config_matrix(Thrusters, COB, 0,0,0);
    B = T'/(T*T');
    Tau = [[10 0 0 0 0 0]', [0 10 0 0 0 0]', [0 0 10 0 0 0]',...
          [0 0 0 10 0 0]', [0 0 0 0 10 0]', [0 0 0 0 0 10]'];

    cost = 0;
    for i = 1:6
        u = B*Tau(:,i);
        %disp(u)
        prod = 1;
        for j = 1:numel(u)
            prod = prod*u(j);
        end
        cost = cost + sum(abs(u));
    end

    penalty = 0;
    nThrusters = length(Thrusters);
    for i = 1:nThrusters-1
        for j = i+1:nThrusters
            Thruster_1 = Thrusters(i);
            Thruster_1 = Thruster_1{1};
            Thruster_2 = Thrusters(j);
            Thruster_2 = Thruster_2{1};
            distance = norm(Thruster_1(1:3)-Thruster_2(1:3));
            if distance < 0.1 % Assuming a very small threshold for overlap
                penalty = penalty + 100; % Increment penalty for each overlap
            end
        end
    end

    cost = cost + penalty;
    if cost>1000
        cost = 1000;
    end
end