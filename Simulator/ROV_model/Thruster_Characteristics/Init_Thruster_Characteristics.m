%% Thruster characteristics
% Based on the code provided by MathWorks Student Competitions Team at:

% MathWorks Student Competitions Team (2024). 
% MATLAB and Simulink Robotics Arena : From Data to Model 
% (https://www.mathworks.com/matlabcentral/fileexchange/65919-matlab-and-simulink-robotics-arena-from-data-to-model), 
% MATLAB Central File Exchange. 

g = 9.82;
filenameSine = 'T200_Sine_0-10_Hz_1600-1900_us.csv';
filenameSquare = 'T200_Square_0-10_Hz_1600-1900_us.csv';
startRow = 2;
endRow = 15002;
Ts = 0.002; 
[input_sn,force_sn] = importData(filenameSine, startRow, endRow);
[input_sq,force_sq] = importData(filenameSquare, startRow, endRow);
sine = iddata(force_sn*0.45359237*g,(input_sn-1500)/400,Ts);
square = iddata(force_sq*0.45359237*g,(input_sq-1500)/400,Ts);

clearvars filenameSine filenameSquare startRow endRow

%% Pre-processing

% Remove Means 
sined = detrend(sine,0);
squared = detrend(square,0);

% Filter 
sinedf = idfilt(sined,5,0.07394);
squaredf = idfilt(squared,5,0.064399);

sinef = idfilt(sine,5,0.07394);
squaref = idfilt(square,5,0.3);

figure(1) 
subplot(2,1,1)
plot(sinef)
title("Sine wave 0-10 Hz",'FontSize', 18)
ylabel('Force [N]','FontSize', 16)
xlabel('Time [s]','FontSize', 16)
xlim([0,5])
ax = gca;
ax.FontSize = 16;

subplot(2,1,2)
plot(squaref)
title("Square wave 0-10 Hz",'FontSize', 18)
ylabel('Force [N]','FontSize', 16)
xlabel('Time [s]','FontSize', 16)
xlim([0,5])
ax = gca;
ax.FontSize = 16;

%% Identify Transfer function model from the video 

Options = tfestOptions;
Options.Display = 'on';
Options.WeightingFilter = [];

tf1 = tfest(squaredf,3,1,Options);

K = minreal(tf1/dcgain(tf1));

%% Plot thruster Characteristics for different voltages
filename = 'T200-Public-Performance-Data-10-20V-September-2019.xlsx'; 
sheetNames = ["10 V","12 V","14 V","16 V","18 V","20 V"];
colours = ["r-", "b-", "k-", "g-", "m-","c-"];
figure(2)
hold on
for i =1:numel(sheetNames)
    Thruster_data = readtable(filename, 'Sheet', sheetNames(i));
    Force = abs(Thruster_data.Force_KgF_*g);
    u = (Thruster_data.PWM__s_-1500)/400;
    plot(u,Force,colours(i),'LineWidth',2)
end
grid on
ylim([-5, 70]);
title("T200 Thruster Force",'FontSize', 18)
xlabel('Input','FontSize', 16)
ylabel('Force [N]','FontSize', 16)
legend("10 V","12 V","14 V","16 V","18 V","20 V",'Orientation','horizontal')
ax = gca;
ax.FontSize = 16;
hold off
u_2 = u(101:end);
Force_2 = Force(101:end);

%% Fit polynomial

filename = 'T200-Public-Performance-Data-10-20V-September-2019.xlsx'; % Provide the full path to your Excel file
sheetName = '14 V';
Thruster_data = readtable(filename, 'Sheet', sheetName);
Force = Thruster_data.Force_KgF_*g;
u = (Thruster_data.PWM__s_-1500)/400;
p = polyfit(u, Force, 9);
p(10)=0;
yfit = polyval(p, u);

figure(3)
plot(u, abs(Force), '.', u, abs(yfit), '-','MarkerSize',10,'LineWidth', 2);
xlabel('Input (u)','FontSize', 16);
ylabel('Force [N]','FontSize', 16);
title('Linear Regression on 14 V characteristics','FontSize', 18);
ax = gca;
ax.FontSize = 16;
legend('Data', 'Linear Fit');
grid on
ylim([0, 45]);

%% simulation

timeVector = 0:0.002:(length(input_sn)-1)*0.002;
input_sq_TimeSeries = timeseries((input_sq-1500)/400, timeVector);
force_sq_TimeSeries = timeseries(squaref.y, timeVector);

input_sn_TimeSeries = timeseries((input_sn-1500)/400, timeVector);
force_sn_TimeSeries = timeseries(sinef.y, timeVector);

