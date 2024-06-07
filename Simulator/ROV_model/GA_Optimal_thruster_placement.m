%% Thruster Placements and orientations
clc
clear
close all

m = 13.5;
dim = [0.457 0.575 0.378];
[I, COB] = InertiaDyadic(3,m,dim);

if COB(3) == 0
    reduced = 1;
else
    reduced = 0;
end

Gram_sys = CreateROVLin(m, I, COB, dim, reduced);

h = dim(3);
l = dim(1);
w = dim(2);

NumPositions = 5;
NumSteps = 19;
Num_trusters = 6;

PopSize = 200;
MaxGen = 20000000;

pos_rot_total = GeneratePosRot(NumPositions,NumSteps);
num_pos_rot_total = length(pos_rot_total);

%%

Thrusters_man{1} = [ 0.5*l,  0.5*w, 0.5*h, 0, 0, -1];
Thrusters_man{2} = [-0.5*l,  0.5*w, 0.5*h, 0, 0, -1];
Thrusters_man{3} = [ 0.5*l, -0.5*w, 0.5*h, 0, 0, -1];
Thrusters_man{4} = [-0.5*l, -0.5*w, 0.5*h, 0, 0, -1];

Thrusters_man{5} = [ 0.5*l,  0.5*w, -COB(3),  1, -1, 0];
Thrusters_man{6} = [-0.5*l,  0.5*w, -COB(3), -1, -1, 0];
Thrusters_man{7} = [ 0.5*l, -0.5*w, -COB(3),  1,  1, 0];
Thrusters_man{8} = [-0.5*l, -0.5*w, -COB(3), -1,  1, 0];

%%
% GA parameters
initialPopulation = randi([2, num_pos_rot_total], PopSize, Num_trusters);
initialPopulation(1,:) = closestIndexes;
lb = ones(1, Num_trusters);
ub = num_pos_rot_total*ones(1, Num_trusters);
rng default

options = optimoptions('ga',...
                       'PopulationSize', PopSize, ...
                       'MaxGenerations', MaxGen,...
                       'InitialPopulationMatrix',initialPopulation,...
                       'CrossoverFraction', 0.5, ...
                       'PlotFcn','gaplotbestf');

[optimSol, fval] = ga(@(u)CostFunction(l, w, h, COB, pos_rot_total, u, Gram_sys,reduced), Num_trusters, [], [], [], [], lb, ub, [], 1:Num_trusters, options);
%%
fig = figure(2);
Thrusters = GenerateThrusters(pos_rot_total, optimSol, l, w, h);
Plot_ROV(l,w,h, COB, Thrusters) 


T = Thrust_config_matrix(Thrusters, COB, 0,0,0);
Best_Cost = CostFunction(l,w,h, COB,pos_rot_total, optimSol, Gram_sys, reduced);

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
Manual_Cost_1 = Cost(Thrusters_man, COB, Gram_sys, reduced);
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
Manual_Cost_2 = Cost(Thrusters_man, COB, Gram_sys, reduced);
disp('Cost of Blue ROV type placement:')
disp(Manual_Cost_2)

%%
% Saving the data
%folderPath = 'C:\Users\Tomas\MATLAB Drive\Ottos_Matlab_eventyr\ROV_model\GA_Results\Optimal_placement_test'; % Tomas Desktop
 folderPath = 'C:\Users\Ottol\Documents\UIS\Master Thesis\Matlab_Master\Ottos_Matlab_eventyr\ROV_model\GA_Results\Test_weighted';  % Change this to your desired path
% Create the folder if it doesn't exist
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

dataFileName = fullfile(folderPath, 'ga_data.mat');
gafigureFileName = fullfile(folderPath, 'ga_figure.fig');
ROVfigureFileName = fullfile(folderPath, 'ROV_figure.fig');
save(dataFileName, 'Gram_sys', 'Num_trusters','NumPositions', "NumSteps", ...
    'num_pos_rot_total', "initialPopulationSize", 'PopSize', 'MaxGen',...
    'optimSol','T', 'Best_Cost', 'Thrusters', 'Manual_T_1', 'Manual_T_2');  % List other variables as needed
saveas(1, gafigureFileName);
saveas(fig, ROVfigureFileName);

%%
function cost = CostFunction(l,w,h, COB, pos_rot_total, u, Gram_sys, reduced)
    
    Thrusters = GenerateThrusters(pos_rot_total, u, l, w, h); 
    cost = Cost(Thrusters, COB, Gram_sys, reduced);
   
end

function cost = Cost(Thrusters, COB, Gram_sys, reduced)

    opt = gramOptions('FreqIntervals',[0 320]);
    T = Thrust_config_matrix(Thrusters, COB, 0,0,0);
    [~,tcol] = size(T);
    transcost = 0;
    rotcost = 0;
    if reduced
        B = Gram_sys.ROV.M\T;
        [brow,~] = size(B);
        D = zeros(brow,tcol);
        W = diag([1 1 1 1 1 1]);
    else
        B = [Gram_sys.ROV.M\T;zeros([2,tcol])];
        [brow,~] = size(B);
        D = zeros(brow,tcol);
        W= diag([1 1 1 1 1 1 1 1]);
        W(7:8,7:8) = 0.001*eye(2);
    end
        
        
    %
    for i = 1:length(Gram_sys.linearizations)
        sys = ss(Gram_sys.linearizations{i}.A,B,Gram_sys.linearizations{i}.C,D);
        [V,Deig] = eig(W*gram(sys,'c', opt)*W');
        % Correlate eigenvalues with dominant direction
        Trans_eig = zeros(1,6);  
        for column = 1:size(V, 2)  
            [~, maxIndex] = max(abs(V(:, column)));  
            if ismember(maxIndex, 1:6)
                Trans_eig(maxIndex) = Deig(column,column);
            end
        end
        % Calculate cost
        transavg = sum(Trans_eig(1:3))/3;
        transcost = transcost + sum(Trans_eig(1:3)) - (abs(Trans_eig(1)-transavg)+abs(Trans_eig(2)-transavg)+abs(Trans_eig(3)-transavg));
        rotavg = sum(Trans_eig(4:6))/3;
        rotcost = rotcost + sum(Trans_eig(4:6)) - (abs(Trans_eig(4)-rotavg)+abs(Trans_eig(5)-rotavg)+abs(Trans_eig(6)-rotavg));
    end

    % Penalizing overlapping positions
    penalty = 0;
    nThrusters = length(Thrusters);
    for i = 1:nThrusters-1
        for j = i+1:nThrusters
            Thruster_1 = Thrusters(i);
            Thruster_1 = Thruster_1{1};
            Thruster_2 = Thrusters(j);
            Thruster_2 = Thruster_2{1};
            distance = norm(Thruster_1(1:3)-Thruster_2(1:3));
            if distance < 0.1 
                penalty = penalty + 1; 
            end
        end
    end

    cost = penalty - 50*transcost - 1*rotcost;
end