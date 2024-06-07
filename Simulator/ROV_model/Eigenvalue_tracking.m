clc
clear
close all

m = 13.5;
dim = [0.457 0.575 0.378];
[I, COB] = InertiaDyadic(3,m,dim);

Reduced = 0;

ROV = initializeModelMatrices(m, I, COB,dim,0);

%%

numSteps = 500; 
finalValue = 3;
state = 'u';
DrawLive = false;

eta0 = [0.0 0.0 0.0 0.0 0.0 0.0]';
v0_initial = [0 0 0 0 0 0]';
sys = linearize_ROV(ROV, v0_initial, eta0, Reduced);
[prev_eigVecs, prev_eigVals] = eig(sys.A);
prev_eigenvalues = diag(prev_eigVals);
numEigenvalues = length(prev_eigenvalues);

% Normalize previous eigenvectors
prev_eigVecs = prev_eigVecs ./ vecnorm(prev_eigVecs);

% Initialize a colormap, one color for each eigenvalue trajectory
colors = lines(numEigenvalues);
LW = 1:2/numEigenvalues:3;

figure(1);
hold on;

firstCrossing = true;

% Iterate through your v0 values
for i = 1:numSteps
    if state == 'u'
    v0 = [(i-1) * finalValue / (numSteps - 1) 0 0 0 0 0]';
    k = 1;
    elseif state == 'v'
    v0 = [0 (i-1) * finalValue / (numSteps - 1) 0 0 0 0]'; 
    k = 2;
    elseif state == 'w'
    v0 = [0 0 (i-1) * finalValue / (numSteps - 1) 0 0 0]'; 
    k = 3;
    elseif state == 'p'
    v0 = [0 0 0 (i-1) * finalValue / (numSteps - 1) 0 0]'; 
    k = 4;
    elseif state == 'q'
    v0 = [0 0 0 0 (i-1) * finalValue / (numSteps - 1) 0]'; 
    k = 5;
    elseif state == 'r'
    v0 = [0 0 0 0 0 (i-1) * finalValue / (numSteps - 1)]'; 
    k = 6;
    end
    sys = linearize_ROV(ROV, v0, eta0, Reduced);
    [eigVecs, eigVals] = eig(sys.A);
    current_eigenvalues = diag(eigVals);

    % Normalize eigenvectors
    eigVecs = eigVecs ./ vecnorm(eigVecs);

    % Check if any eigenvalue has a positive real part
    if firstCrossing && any(real(current_eigenvalues) >= 0)
        firstCrossing = false;  % Prevent multiple printouts
        v0_unstable = v0;
        fprintf('v0 leading to right half-plane crossing: [%f %f %f %f %f %f]\n', v0);
        disp('Eigenvalues at crossing:');
        disp(current_eigenvalues);

        % Find and display the corresponding eigenvectors for unstable eigenvalues
        unstableIndices = find(real(current_eigenvalues) >= 0);
        disp('Corresponding eigenvectors (directions) for unstable eigenvalues:');
        for idx = unstableIndices'
            disp(['Eigenvector for eigenvalue ', num2str(current_eigenvalues(idx)) ':']);
            disp(eigVecs(:, idx));
        end
    end

    % % Check if any eigenvalue has a positive real part
    % if firstCrossing && any(real(current_eigenvalues) >= 0)
    %     firstCrossing = false; % Prevent multiple printouts
    %     fprintf('v0 leading to right half-plane crossing: [%f %f %f %f %f %f]\n', v0);
    %     disp('Eigenvalues at crossing:');
    %     disp(current_eigenvalues);
    % end

    costMatrix = zeros(numEigenvalues, numEigenvalues);
    for m = 1:numEigenvalues
        for n = 1:numEigenvalues
            % Ensuring the cost is a real value by taking the absolute value
            % and adjusting the calculation to avoid complex values
            costMatrix(m, n) = abs(real(prev_eigenvalues(m) - current_eigenvalues(n))) + (1 - abs(dot(prev_eigVecs(:, m), eigVecs(:, n))));
        end
    end

    % Explicitly converting to double, just to be safe
    costMatrix = double(costMatrix);

    [assignment, ~] = matchpairs(costMatrix, max(costMatrix(:)) + 1);

    % Plot the trajectories
    for j = 1:size(assignment, 1)
        prev_index = assignment(j, 1);
        current_index = assignment(j, 2);
        prev_val = prev_eigenvalues(prev_index);
        current_val = current_eigenvalues(current_index);

        % Initial points
        if i == 1
            plot(real(prev_val), imag(prev_val),'x', 'MarkerEdgeColor', colors(prev_index, :), 'MarkerSize', 10, 'LineWidth', 2);
        end

        % Lines
        line([real(prev_val), real(current_val)], [imag(prev_val), imag(current_val)], 'Color', colors(prev_index, :), 'LineStyle', ':', 'LineWidth', LW(prev_index));

        % Final points
        if i == numSteps
            plot(real(current_val), imag(current_val), 'o', 'MarkerFaceColor', colors(prev_index, :), 'MarkerEdgeColor', colors(prev_index, :));
        end

        % Update for the next iteration
        prev_eigenvalues(prev_index) = current_val;
        prev_eigVecs(:, prev_index) = eigVecs(:, current_index);
    end

    if DrawLive
        drawnow;
        pause(0.1);  % Adjust the pause as needed for the live drawing speed
    end
end
if firstCrossing
    plotTitle = sprintf('Trajectories of Eigenvalues with %s varying from 0 to %.2f', state, finalValue);
else
    plotTitle = sprintf('Trajectories of Eigenvalues with %s varying from 0 to %.2f \n Becoming unstable at %s = %.2f', state, finalValue, state, v0_unstable(k));
end
title(plotTitle,'FontSize', 18);
xlabel('Real Part','FontSize', 16);
ylabel('Imaginary Part','FontSize', 16);
grid on;
%axis equal;
ax = gca;
ax.FontSize = 16;
hold off;


% eta0 = [0.0 0.0 0.0 0.0 0.0 0.0]';
% v0_initial = [0 0 0 0 0 0]';
% prev_eigenvalues = eig(linearize_ROV(ROV, v0_initial, eta0, 0));
% numEigenvalues = length(prev_eigenvalues);
% 
% 
% % Initialize a colormap, one color for each eigenvalue trajectory
% colors = lines(numEigenvalues);
% LW = 1:2/numEigenvalues:3;
% 
% figure(1);
% hold on;
% 
% firstCrossing = true;
% 
% % Iterate through your v0 values
% for i = 1:numSteps
%     if state == 'u'
%     v0 = [(i-1) * finalValue / (numSteps - 1) 0 0 0 0 0]';
%     elseif state == 'v'
%     v0 = [0 (i-1) * finalValue / (numSteps - 1) 0 0 0 0]'; 
%     elseif state == 'w'
%     v0 = [0 0 (i-1) * finalValue / (numSteps - 1) 0 0 0]'; 
%     elseif state == 'p'
%     v0 = [0 0 0 (i-1) * finalValue / (numSteps - 1) 0 0]'; 
%     elseif state == 'q'
%     v0 = [0 0 0 0 (i-1) * finalValue / (numSteps - 1) 0]'; 
%     elseif state == 'r'
%     v0 = [0 0 0 0 0 (i-1) * finalValue / (numSteps - 1)]'; 
%     end
%     current_eigenvalues = eig(linearize_ROV(ROV, v0, eta0, 0));
% 
%     % Check if any eigenvalue has a positive real part
%     if firstCrossing && any(real(current_eigenvalues) >= 0)
%         firstCrossing = false; % Prevent multiple printouts
%         fprintf('v0 leading to right half-plane crossing: [%f %f %f %f %f %f]\n', v0);
%         disp('Eigenvalues at crossing:');
%         disp(current_eigenvalues);
%     end
% 
%     % Form the cost matrix
%     costMatrix = abs(prev_eigenvalues - current_eigenvalues.');
% 
%     % Solve the assignment problem
%     [assignment, ~] = matchpairs(costMatrix, max(costMatrix(:)) + 1);
% 
%     % Plot the initial points and trajectories
%     for j = 1:size(assignment, 1)
%         prev_index = assignment(j, 1);
%         current_index = assignment(j, 2);
%         prev_val = prev_eigenvalues(prev_index);
%         current_val = current_eigenvalues(current_index);
% 
%         % Plot the initial points in the color of the trajectory
%         if i == 1
%             plot(real(prev_val), imag(prev_val), 'o', 'MarkerFaceColor', colors(prev_index, :), 'MarkerEdgeColor', colors(prev_index, :));
%         end
% 
%         % Plot a line from the previous to the current eigenvalue
%         line([real(prev_val), real(current_val)], [imag(prev_val), imag(current_val)], 'Color', colors(prev_index, :), 'LineStyle', ':', 'LineWidth',LW(prev_index));
% 
%         % Update the previous eigenvalues for the next iteration
%         prev_eigenvalues(prev_index) = current_val;
% 
%         % Plot the final points as 'X' in the color of the trajectory
%         if i == numSteps
%             plot(real(current_val), imag(current_val), 'x', 'MarkerEdgeColor', colors(prev_index, :), 'MarkerSize', 10, 'LineWidth', 2);
%         end
%     end
%     if DrawLive
%         drawnow;
%         pause(0.1)
%     end
% end
% 
% title('Trajectories of Eigenvalues');
% xlabel('Real Part');
% ylabel('Imaginary Part');
% grid on;
% axis equal;
% hold off;




function linear_model = linearize_ROV(ROV_parameters, Velocity, Position, Reduced)
R = ROV_parameters;
syms u v w p q r x y z phi theta pzi real
if Reduced
    A = double(subs(R.As, ...
                        [u v w p q r x y z phi theta pzi], ...
                        [Velocity' Position']));
    B = R.Bs;    C = eye(6);    D = zeros(6,8);
else
    A = double(subs(R.Ar, ...
                        [u v w p q r x y z phi theta pzi], ...
                        [Velocity' Position']));
    B = R.Br;    C = eye(8);    D = zeros(8);
end
linear_model = ss(A,B,C,D);
% disp(A)
% disp(B)
end



