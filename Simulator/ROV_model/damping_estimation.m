% damnping_estimation
function [Ds, Dm] = damping_estimation(dim, Ap, I, Ma, deltaBG, weight)
% deltaBG = 0.04;
% dim = [0.378 0.575 0.457];
HEIGHT = dim(3)*1e3;
WIDTH = dim(2)*1e3;
LENGTH = dim(1)*1e3;
% Ap = [WIDTH*LENGTH HEIGHT*WIDTH LENGTH*HEIGHT]*0.7;
PT = Ap(1);
PF = Ap(2);
PS = Ap(3);
rho = 997;
% I = diag([0.26, 0.23, 0.37]);
I44 = I(1,1);
I55 = I(2,2);
% Ma = added_mass(dim, Ap);
A44 = Ma(4,4);
A55 = Ma(5,5);
Rc = deltaBG*weight*1e3; % Restoring coefficient from Eidsvik Master Thesis
if Rc == 0
    Rc = 3000;
end

CpXY=PT/(LENGTH*WIDTH);     % Projected Area Coefficient XY
CpYZ=PF/(HEIGHT*WIDTH);     % Projected Area Coefficient YZ
CpXZ=PS/(LENGTH*HEIGHT);    % Projected Area Coefficient XZ

%Drag Coefficients (2D)
Data2D =[0.5,2.5;1.5,1.8;2.5,1.4;6,0.89];
% Blevins table 10.20 page 343
Drag2D=spline(Data2D(:,1),Data2D(:,2));
%Drag Coefficients (3D)
Data3D =[0,1.25;0.5,1.25;1,1.15;1.5,0.97;2,0.87;...
2.5,0.9;3,0.93;4,0.95;5,0.95];
Drag3D=spline(Data3D(:,1),Data3D(:,2));

%Nonlinear damping
% Surge 3D
LD=LENGTH/((HEIGHT+WIDTH)/2);
BQ3D =ppval(Drag3D,(LD));
% Surge 2D
LD=LENGTH/WIDTH;
BQ2D=ppval(Drag2D,(LD));
lambda = BQ3D/BQ2D;

% Final Surge nonlinear damping
LD=LENGTH/((HEIGHT+WIDTH)/2);
BQ(1,1)=0.5*rho*ppval(Drag2D,(LD))*HEIGHT*WIDTH*10^-6*CpYZ*lambda;
% Sway
LD=WIDTH/HEIGHT;
BQ(2,2)=rho*0.5*ppval(Drag2D,(LD))*LENGTH*HEIGHT*10^-6*CpXZ*lambda;
% Heave
LD=HEIGHT/WIDTH;
ppval(Drag2D,(LD));
BQ(3,3)=rho*0.5*ppval(Drag2D,(LD))*LENGTH*WIDTH*10^-6*CpXY*lambda;

% Roll
LD=WIDTH/(HEIGHT/2);
Fh=rho*(1/6)*ppval(Drag2D,(LD))*(HEIGHT/2)*LENGTH*10^-6*CpXZ*lambda;
Mh=Fh*(3/4)*((HEIGHT/2)*10^-3)^3;
LD=HEIGHT/(WIDTH/2);
Fv=rho*(1/6)*ppval(Drag2D,(LD))*(WIDTH/2)*LENGTH*10^-6*CpXY*lambda;
Mv=Fv*(3/4)*((WIDTH/2)*10^-3)^3;
BQ(4,4)= (2*Mv+2*Mh);

% Pitch
LD=LENGTH/(HEIGHT/2);
Fh=rho*(1/6)*ppval(Drag2D,(LD))*(HEIGHT/2)*WIDTH*10^-6*CpYZ*lambda;
Mh=Fh*(3/4)*((HEIGHT/2)*10^-3)^3;
LD=HEIGHT/(LENGTH/2);
Fv=rho*(1/6)*ppval(Drag2D,(LD))*(LENGTH/2)*WIDTH*10^-6*CpXY*lambda;
Mv=Fv*(3/4)*((LENGTH/2)*10^-3)^3;
BQ(5,5)= (2*Mv+2*Mh);

% Yaw
LD=LENGTH/(WIDTH/2);
Fh=rho*(1/6)*ppval(Drag2D,(LD))*(WIDTH/2)*HEIGHT*10^-6*CpYZ*lambda;
Mh=Fh*(3/4)*((WIDTH/2)*10^-3)^3;
LD=WIDTH/(LENGTH/2);
Fv=rho*(1/6)*ppval(Drag2D,(LD))*(LENGTH/2)*HEIGHT*10^-6*CpXZ*lambda;
Mv=Fv*(3/4)*((LENGTH/2)*10^-3)^3;
BQ(6,6)= (2*Mv+2*Mh);

%Roll and Pitch
BL(4,4)= 2*0.025*(I44+A44)*sqrt(Rc/(I44+A44));
BL(5,5)= 2*0.025*(I55+A55)*sqrt(Rc/(I55+A55));
lambda0=0.16;
lambda1=BL(5,5)/BQ(5,5);
%Surge,Sway, heave and yaw
BL(1,1)=BQ(1,1)*lambda0;
BL(2,2)=BQ(2,2)*lambda0;
BL(3,3)=BQ(3,3)*lambda0;
BL(6,6)=BQ(6,6)*lambda1;

Ds = BL;
Dm = BQ;
end


