function PosRot = GeneratePosRot(NumPositions,NumSteps)


    pos_rot_ps = Generate_pos_rot('ps', NumPositions, NumSteps, 0);
    pos_rot_fa = Generate_pos_rot('fa', NumPositions, NumSteps, 0);
    pos_rot_bt = Generate_pos_rot('bt', NumPositions, NumSteps, 0);
    pos_rot_edges = Generate_pos_rot_edges(NumPositions, NumSteps, 0);
    pos_rot_corners = Generate_pos_rot_corners(NumSteps);
    
    PosRot = [pos_rot_ps, pos_rot_fa, pos_rot_bt, pos_rot_edges, pos_rot_corners];
    
    % length(pos_rot_ps)
    % length(pos_rot_fa)
    % length(pos_rot_bt)
    % length(pos_rot_edges)
    % length(pos_rot_corners)

end


function y = Generate_pos_rot(sides, Num_positions, numberOfSteps, include_edges)
l = 1;
h = 1;
w = 1;
positions = {};
rotations = {};

if sides == 'ps' %port/starboard
   a = w; b = l; c = h;
elseif sides == 'fa' % fore/aft
   a = l; b = h; c = w;
elseif sides == 'bt' % bottom/top
   a = h; b = w; c = l;
end

if include_edges
        l1 = linspace(-b/2,b/2,Num_positions);
        l2 = linspace(-c/2,c/2,Num_positions);
    
    else
        l1 = linspace(-b/2,b/2,Num_positions);
        l1 = l1(2:end-1);
        l2 = linspace(-c/2,c/2,Num_positions);
        l2 = l2(2:end-1);
        
    end
    




for i = [-a/2, a/2]
    for j = l1
        for k = l2
            if sides == 'ps' %port/starboard
               positions{end+1} = [j, i, k];
            elseif sides == 'fa' % fore/aft
               positions{end+1} = [i, k, j];
            elseif sides == 'bt' % bottom/top
               positions{end+1} = [k, j, i];
            end
        end
    end
end
    pos = positions;

angleStep =  pi/2/ numberOfSteps;

for i = 1:(numberOfSteps*4)
    angle = (i - 1) * angleStep;
    
    if sides == 'ps' %port/starboard
       rotations{end+1} = [round(cos(angle),3) , 0, round(sin(angle),3)];
    elseif sides == 'fa' % fore/aft
       rotations{end+1} = [0, round(sin(angle),3), round(cos(angle),3) ];
    elseif sides == 'bt' % bottom/top
       rotations{end+1} = [round(sin(angle),3), round(cos(angle),3) , 0];
    end

end
    rot = rotations;
    comb = {};
    for i = 1:length(pos)
    % Loop over each vector in B
        for j = 1:length(rot)
            % Concatenate current vectors from A and B
            combinedVector = [pos{i}, rot{j}]; % This creates a 6x1 vector
    
            % Append the combined vector to the cell array C
            comb{end+1} = combinedVector;
        end
    end

    y = comb;
end

function y = Generate_pos_rot_edges(Num_positions, numberOfSteps, include_corners)
    l = 1;
    h = 1;
    w = 1;
    
    R_x = @(theta)([1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)]);
    R_y = @(theta)([cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)]);
    R_z = @(theta)([cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1]);
    
    if include_corners
        l1 = linspace(-h/2,h/2,Num_positions);
        l2 = linspace(-l/2,l/2,Num_positions);
        l3 = linspace(-w/2,w/2,Num_positions);
    
    else
        l1 = linspace(-h/2,h/2,Num_positions);
        l1 = l1(2:end-1);
        l2 = linspace(-l/2,l/2,Num_positions);
        l2 = l2(2:end-1);
        l3 = linspace(-w/2,w/2,Num_positions);
        l3 = l3(2:end-1);
    end
    
    ang1 = linspace(0,pi/2,numberOfSteps);
    % ang1 = ang1(2:end-1);

    ang2 = linspace(0,pi,numberOfSteps*2);
    % ang2 = ang2(2:end-1);


    positions_pos_pos = {};
    positions_neg_pos = {};
    positions_neg_neg = {};
    positions_pos_neg = {};

    rotations_pos_pos = {};
    rotations_neg_pos = {};
    rotations_neg_neg = {};
    rotations_pos_neg = {};

    for i = l1
        positions_pos_pos{end+1} = [l/2, w/2, i];
        positions_neg_pos{end+1} = [-l/2, w/2, i];
        positions_neg_neg{end+1} = [-l/2, -w/2, i];
        positions_pos_neg{end+1} = [l/2, -w/2, i];

    end


    for i = ang1
        for j = ang2
        rotations_pos_pos{end+1} = R_z(i)*R_x(j)*[0,-1,0]';
        rotations_neg_pos{end+1} = R_z(i)*R_y(j)*[1,0,0]';
        rotations_neg_neg{end+1} = R_z(i)*R_x(j)*[0,1,0]';
        rotations_pos_neg{end+1} = R_z(i)*R_y(j)*[-1,0,0]';
        
        end
    end


    comb1 = {};
    for i = 1:length(positions_pos_pos)
    % Loop over each vector in B
        for j = 1:length(rotations_pos_pos)
            % Concatenate current vectors from A and B
            combined_pos_pos = [positions_pos_pos{i}, rotations_pos_pos{j}']; % This creates a 6x1 vector
            combined_neg_pos = [positions_neg_pos{i}, rotations_neg_pos{j}'];
            combined_neg_neg = [positions_neg_neg{i}, rotations_neg_neg{j}'];
            combined_pos_neg = [positions_pos_neg{i}, rotations_pos_neg{j}'];

            % Append the combined vector to the cell array C
            comb1{end+1} = combined_pos_pos;
            comb1{end+1} = combined_neg_pos;
            comb1{end+1} = combined_neg_neg;
            comb1{end+1} = combined_pos_neg;
        end
    end

    positions_pos_pos = {};
    positions_neg_pos = {};
    positions_neg_neg = {};
    positions_pos_neg = {};

    rotations_pos_pos = {};
    rotations_neg_pos = {};
    rotations_neg_neg = {};
    rotations_pos_neg = {};


    for i = l2
        positions_pos_pos{end+1} = [i, w/2, h/2];
        positions_neg_pos{end+1} = [i, -w/2, h/2];
        positions_neg_neg{end+1} = [i, -w/2, -h/2];
        positions_pos_neg{end+1} = [i, w/2, -h/2];

    end

    for i = ang1
        for j = ang2
        rotations_pos_pos{end+1} = R_x(i)*R_y(j)*[0,0,-1]';
        rotations_neg_pos{end+1} = R_x(i)*R_z(j)*[0,1,0]';
        rotations_neg_neg{end+1} = R_x(i)*R_y(j)*[0,0,1]';
        rotations_pos_neg{end+1} = R_x(i)*R_z(j)*[0,-1,0]';

        end
    end

    comb2 = {};
    for i = 1:length(positions_pos_pos)
    % Loop over each vector in B
        for j = 1:length(rotations_pos_pos)
            % Concatenate current vectors from A and B
            combined_pos_pos = [positions_pos_pos{i}, rotations_pos_pos{j}']; % This creates a 6x1 vector
            combined_neg_pos = [positions_neg_pos{i}, rotations_neg_pos{j}'];
            combined_neg_neg = [positions_neg_neg{i}, rotations_neg_neg{j}'];
            combined_pos_neg = [positions_pos_neg{i}, rotations_pos_neg{j}'];

            % Append the combined vector to the cell array C
            comb2{end+1} = combined_pos_pos;
            comb2{end+1} = combined_neg_pos;
            comb2{end+1} = combined_neg_neg;
            comb2{end+1} = combined_pos_neg;
        end
    end


    positions_pos_pos = {};
    positions_neg_pos = {};
    positions_neg_neg = {};
    positions_pos_neg = {};

    rotations_pos_pos = {};
    rotations_neg_pos = {};
    rotations_neg_neg = {};
    rotations_pos_neg = {};

    for i = l3
        positions_pos_pos{end+1} = [l/2, i, h/2];
        positions_neg_pos{end+1} = [-l/2, i, h/2];
        positions_neg_neg{end+1} = [-l/2, i, -h/2];
        positions_pos_neg{end+1} = [l/2, i, -h/2];

    end


    for i = ang1
        for j = ang2
        rotations_pos_pos{end+1} = R_y(i)*R_z(j)*[-1,0,0]';
        rotations_neg_pos{end+1} = R_y(i)*R_x(j)*[0,0,-1]';
        rotations_neg_neg{end+1} = R_y(i)*R_z(j)*[1,0,0]';
        rotations_pos_neg{end+1} = R_y(i)*R_x(j)*[0,0,1]';

        end
    end

    comb3 = {};
    for i = 1:length(positions_pos_pos)
    % Loop over each vector in B
        for j = 1:length(rotations_pos_pos)
            % Concatenate current vectors from A and B
            combined_pos_pos = [positions_pos_pos{i}, rotations_pos_pos{j}']; % This creates a 6x1 vector
            combined_neg_pos = [positions_neg_pos{i}, rotations_neg_pos{j}'];
            combined_neg_neg = [positions_neg_neg{i}, rotations_neg_neg{j}'];
            combined_pos_neg = [positions_pos_neg{i}, rotations_pos_neg{j}'];

            % Append the combined vector to the cell array C
            comb3{end+1} = combined_pos_pos;
            comb3{end+1} = combined_neg_pos;
            comb3{end+1} = combined_neg_neg;
            comb3{end+1} = combined_pos_neg;
        end
    end


    y = [comb1,comb2, comb3];
   
      
end

function y = Generate_pos_rot_corners(numberOfSteps)
    l = 1;
    h = 1;
    w = 1;
    
    R_x = @(theta)([1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)]);
    R_y = @(theta)([cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)]);
    R_z = @(theta)([cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1]);
    
    ang1 = linspace(0,pi/2,numberOfSteps);
    %ang1 = ang1(2:end-1);

    ang2 = linspace(0,pi/2,numberOfSteps);
    %ang2 = ang2(2:end-1);

    ang3 = linspace(-pi,pi,numberOfSteps*4);
    %ang3 = ang3(2:end-1);


    positions_pos_pos = {[l/2, w/2, -h/2]};
    positions_neg_pos = {[-l/2, w/2, -h/2]};
    positions_neg_neg = {[-l/2, -w/2, -h/2]};
    positions_pos_neg = {[l/2, -w/2, -h/2]};

    rotations_pos_pos = {};
    rotations_neg_pos = {};
    rotations_neg_neg = {};
    rotations_pos_neg = {};

   
    for i = ang1
        for k = ang2
            for j = ang3
                rotations_pos_pos{end+1} = R_z(i)*R_y(k)*R_x(j)*[0,-1,0]';
                rotations_neg_pos{end+1} = R_z(i)*R_x(-k)*R_y(j)*[1,0,0]';
                rotations_neg_neg{end+1} = R_z(i)*R_y(-k)*R_x(j)*[0,1,0]';
                rotations_pos_neg{end+1} = R_z(i)*R_x(k)*R_y(j)*[-1,0,0]';
            end
        end
    end

    comb1 = {};
    for i = 1:length(positions_pos_pos)
    % Loop over each vector in B
        for j = 1:length(rotations_pos_pos)
            % Concatenate current vectors from A and B
            combined_pos_pos = [positions_pos_pos{i}, rotations_pos_pos{j}']; % This creates a 6x1 vector
            combined_neg_pos = [positions_neg_pos{i}, rotations_neg_pos{j}'];
            combined_neg_neg = [positions_neg_neg{i}, rotations_neg_neg{j}'];
            combined_pos_neg = [positions_pos_neg{i}, rotations_pos_neg{j}'];

            % Append the combined vector to the cell array C
            comb1{end+1} = combined_pos_pos;
            comb1{end+1} = combined_neg_pos;
            comb1{end+1} = combined_neg_neg;
            comb1{end+1} = combined_pos_neg;
        end
    end

        positions_pos_pos = {[l/2, w/2, h/2]};
    positions_neg_pos = {[-l/2, w/2, h/2]};
    positions_neg_neg = {[-l/2, -w/2, h/2]};
    positions_pos_neg = {[l/2, -w/2, h/2]};

    rotations_pos_pos = {};
    rotations_neg_pos = {};
    rotations_neg_neg = {};
    rotations_pos_neg = {};

   
    for i = ang1
        for k = ang2
            for j = ang3
                rotations_pos_pos{end+1} = R_z(i)*R_y(-k)*R_x(j)*[0,-1,0]';
                rotations_neg_pos{end+1} = R_z(i)*R_x(k)*R_y(j)*[1,0,0]';
                rotations_neg_neg{end+1} = R_z(i)*R_y(k)*R_x(j)*[0,1,0]';
                rotations_pos_neg{end+1} = R_z(i)*R_x(-k)*R_y(j)*[-1,0,0]';
            end
        end
    end

    comb2 = {};
    for i = 1:length(positions_pos_pos)
    % Loop over each vector in B
        for j = 1:length(rotations_pos_pos)
            % Concatenate current vectors from A and B
            combined_pos_pos = [positions_pos_pos{i}, rotations_pos_pos{j}']; % This creates a 6x1 vector
            combined_neg_pos = [positions_neg_pos{i}, rotations_neg_pos{j}'];
            combined_neg_neg = [positions_neg_neg{i}, rotations_neg_neg{j}'];
            combined_pos_neg = [positions_pos_neg{i}, rotations_pos_neg{j}'];

            % Append the combined vector to the cell array C
            comb2{end+1} = combined_pos_pos;
            comb2{end+1} = combined_neg_pos;
            comb2{end+1} = combined_neg_neg;
            comb2{end+1} = combined_pos_neg;
        end
    end


    y = [comb1, comb2];
      
end

