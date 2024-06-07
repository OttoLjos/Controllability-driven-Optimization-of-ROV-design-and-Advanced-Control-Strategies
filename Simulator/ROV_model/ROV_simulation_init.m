% Initialize and set up ROV simulation
% This file is dependent on the modelsimulator.slx file
% Running this script will open the simulator 
clc, clear

% Fenris: BlueROVHeavy = 0;
m = 24.238;
dim = [0.67 0.465 0.35];
Inertia = diag([0.496, 0.793, 0.870]);
COB = [0 0 -0.0206];

% Setting BlueROVHeavy = 1 ignores the inputted parameters
BlueROVHeavy = 1;

ROV = initializeModelMatrices(m, Inertia, COB, dim, BlueROVHeavy);

v_dot = calcAcceleration(ROV.M,ROV.C,ROV.D,ROV.G);
n_dot = calcNED();
Tau_inv = InvNED();

%% Optimal Solution GD.
ROV.T = [
    0.3042   0.6627  -0.6001  -0.5003   0.8337   0.1106  -0.7670  -0.7248;
    0.4269  -0.5955  -0.2614   0.5996   0.5493   0.3810  -0.4645   0.6522;
    0.8515  -0.4539  -0.7560  -0.6246   0.0571  -0.9179   0.4427   0.2221;
   -0.3093   0.2205  -0.1778  -0.2702   0.1082   0.3503   0.0219   0.2118;
   -0.1485  -0.0035  -0.2635   0.0671  -0.2021  -0.2348   0.2751   0.1136;
    0.1850   0.3266   0.2322   0.2809   0.3652  -0.0553   0.3266   0.3574];

T_inv = ROV.T'/(ROV.T*ROV.T');
%T_inv = ROV.T'/(ROV.T*ROV.T') + rand([8,6])*0.01;

%% Linearize ROV
v_init = [0 0 0 0 0 0];
eta_init = [0 0 0 0 0 0];
linsysROV = linearize_ROV(ROV,v_init', eta_init', 0);
%%  PID Reg
% PID_x = [Kp, Ki, Kd, Tf]
PID_x =     [500.411, 30.5093, -500.8638, 0.032394,110];
PID_y =     [500.7275, 20.1075, -500.8002, 0.019369,110];
PID_z =     [500.7655, 25.0859, -500.5557, 0.026709,158];
PID_phi =   [11.6676, 5.8503, -12.3688, 0.025226,34];
PID_theta = [10.2974, 5.9702, -15.653, 0.016423,19];
PID_psi =   [11.3054, 1.000, -3.8005, 0.087502,30];

%% Full LQR
Q = diag([0 0 0 0 0 0 150000 150000 150000 400 400 500 1000 1000 1000 20 20 10]);
R = eye(6,6);
C_int = [zeros(6), eye(6)];
linsysROV_lqr = ss([linsysROV.A, zeros([12,6]);C_int, zeros(6)], ...
    [linsysROV.B; zeros([6,6])], eye(18), zeros([18,6]));

K_full = lqr(linsysROV_lqr,Q,R);

%% Sliding mode
epsilon =   [0.10, 0.10, 0.10, 0.30, 0.30, 0.20];
b0 =       [39.72, 41.24, 48.28, 3.15, 2.88, 1.95];
c1 =        [2.00, 2.00, 2.00, 2.70, 3.50, 1.30];
c2 =        [0.15, 0.15, 0.15, 0.90, 1.05, 0.50];

SMC(ROV, c1, c2, epsilon, b0)

%% Reduced states LQR
A = linsysROV.A(3:11,3:11);
A([false,false,false,true,true,true,false,false,false],:)=[];
A(:,[false,false,false,true,true,true,false,false,false])=[];
B = linsysROV.B(3:9,:);
B(4,:)=[];
C_int = [zeros(3), eye(3)];

linsysROV_lqr_reduced = ss([A, zeros([6,3]);C_int, zeros(3)], ...
    [B; zeros([3,6])], eye(9), zeros([9,6]));
linsysROV_lqr_reduced_disc = c2d(linsysROV_lqr_reduced, 1/20, 'zoh');

Q = diag([0 0 0  150000 400 400 1000 20 20]);
R = eye(6,6);
K_reduced = lqr(linsysROV_lqr_reduced_disc,Q,R);   

%% Thruster voltage
ThrusterVoltage = 14;
Deadband = 0.08;

if ThrusterVoltage == 10
    p = [-92.7173526709009	-4.21942992652299	225.484550494853	8.63193834440817	-201.940923421530	-6.79855430208119	89.8300206612197	5.42728991500474	5.00491978379944	-0.00];
elseif ThrusterVoltage == 12
    p = [-102.848572308239	-7.53059973031176	248.621713648981	14.4941570056757	-222.949976245640	-9.82587357024907	101.816137648126	6.80722654001185	7.78963212289893	-0.00];
elseif ThrusterVoltage == 14
    p = [-111.940783464042	-6.65861599537878	269.823430407384	13.8659958797022	-243.064927768226	-10.2880091499065	114.063389337106	8.09188399656116	10.6145211900378	-0.00];
elseif ThrusterVoltage == 16
    p =  [-148.575215521583	-6.01951344606880	347.062027953365	14.7939725163702	-299.865931981745	-13.0166356354772	134.177990099770	10.1629233711657	12.7092879704266	-0.00];
elseif ThrusterVoltage == 18
    p = [-162.288059231193	-9.51335388922301	365.972527557958	19.4413549873700	-302.179637889150	-11.4404685775311	136.049916333434	8.61090564659817	14.1180490642860	-0.00];
elseif ThrusterVoltage == 20
    p = [-125.691529893755	-8.50647884753240	275.922008410927	13.4112646958028	-226.567986780204	-3.28875901062827	118.657519470084	6.62648580653865	14.9826070833980	0.00];
end

%% Reference trajectory
omega= 0.0174532925*5;
z_rate = 0.1;
radius = 5;
v_init = [omega omega z_rate 0 0 omega];
eta_init = [radius 0 0 0 0 -pi/2];

%% functions
function linear_model = linearize_ROV(ROV_parameters, Velocity, Position, Reduced)
R = ROV_parameters;
syms u v w p q r x y z phi theta pzi real

if Reduced
    A = double(subs(R.Ar, ...
                        [u v w p q r x y z phi theta pzi], ...
                        [Velocity' Position']));
    B = R.Br;
    C = eye(8);
    D = zeros(8);
else
    A = double(subs(R.A_nl, ...
                            [u v w p q r x y z phi theta pzi], ...
                            [Velocity' Position']));
    B = R.Breg;
    C = eye(12);
    D = zeros(12,6);
end
linear_model = ss(A,B,C,D);
end

function SMC(ROV, c1, c2, epsilon, b0)
syms u v w p q r x y z phi theta pzi x_d y_d z_d phi_d theta_d pzi_d real 
ref = sym('ref', [1 6]);
ref_d = sym('ref_d', [1 6]);
ref_dd = sym('ref_dd', [1 6]);
e_int = sym('e_int', [1 6]);
assume([ref, ref_d, ref_dd, e_int], 'real')

gt = inv(subs(ROV.M_ned,[u v w p q r x y z phi theta pzi],zeros(1,12)));
f = subs(ROV.M_ned\(-ROV.D_ned*[u, v, w, p, q, r]' - ROV.G_ned),[u v w p q r],[x_d y_d z_d phi_d theta_d pzi_d]);
e = [x y z phi theta pzi]-ref;
e_d = [x_d y_d z_d phi_d theta_d pzi_d]-ref_d;

s = e_d+c1.*e+c2.*e_int;

output = -((s)./(abs(s)+epsilon)).*((1./diag(gt))'.*(abs(f' - ref_dd-c1.*e_d-c2.*e))+b0);

matlabFunctionBlock('modelsimulator/SMC/SMC_internal/SMC', output(1), output(2),output(3),output(4),output(5),output(6),s(1),s(2),s(3),s(4),s(5),s(6), ...
        'vars', [x y z phi theta pzi x_d y_d z_d phi_d theta_d pzi_d ref ref_d ref_dd e_int], ...
        'Outputs', {'Tau_1','Tau_2','Tau_3','Tau_4','Tau_5','Tau_6','S_1','S_2','S_3','S_4','S_5','S_6'})
end

