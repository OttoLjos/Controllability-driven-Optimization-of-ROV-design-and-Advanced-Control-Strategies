% Added Mass
function A = added_mass(dim, Ap)
HEIGHT = dim(3)*1e3;
WIDTH = dim(2)*1e3;
LENGTH = dim(1)*1e3;
PT = Ap(1);
PF = Ap(2);
PS = Ap(3);
rho = 997;
A = zeros(6);

% DNV Empirical 3D data
EMP3D=[1,0.68;2,0.36;3,0.24;4,0.19;5,0.15;6,0.14;7,0.11];
CA3D=spline(EMP3D(:,1),EMP3D(:,2));

% DNV Empirical 2D data
EMP2D=[10,1.14,0.125;5,1.21,0.15;2,1.36,0.15;1,1.51,0.234;...
0.5,1.7,0.15;0.2,1.98,0.15;0.1,2.23,0.147];
CA2DT=spline(EMP2D(:,1),EMP2D(:,2));
CA2DR=spline(EMP2D(:,1),EMP2D(:,3));

%Coefficients
H3D=(HEIGHT+WIDTH)/2;       % Averaged Height( For 3D-est)
W3D=H3D;                    % Averaged Width ( For 3D-est)
CpXY=PT/(LENGTH*WIDTH);     % Projected Area Coefficient XY
CpYZ=PF/(HEIGHT*WIDTH);     % Projected Area Coefficient YZ
CpXZ=PS/(LENGTH*HEIGHT);    % Projected Area Coefficient XZ

% Surge 
% 3D
B=LENGTH/H3D;
Ca=ppval(CA3D,(B));
V=LENGTH*H3D^2;
A(1,1)= Ca*V*10^(-9)*rho*(CpYZ)^2*CpXZ*CpXY;
% 2D
B=WIDTH/LENGTH;
Ca=ppval(CA2DT,B);
Ar=pi*((WIDTH*0.5)^2);
A2D=rho*Ca*Ar*10^(-6)*(CpYZ)^2*CpXZ*CpXY;
At=HEIGHT*10^(-3)*A2D;
lambda=sqrt(A(1,1)/At);
A(1,1)=At*lambda;

% Sway
B=LENGTH/WIDTH;
Ca=ppval(CA2DT,B);
Ar=pi*(LENGTH*0.5)^2*10^-6;
A2D=rho*Ca*Ar*CpXZ^2*CpXY*CpYZ;
At=A2D*HEIGHT*10^-3;
A(2,2)=At*lambda;

% Heave
A2D=rho*Ca*Ar*CpXY^2*CpXZ*CpYZ;
At=A2D*WIDTH*10^-3;
A(3,3)=At*lambda;

% Roll
B=HEIGHT/WIDTH;
Ca=ppval(CA2DR,B);
if (B<=1)
    A2D=rho*Ca*pi*(WIDTH*0.5*10^(-3))^4*CpYZ*CpXY*CpXZ;
else
    A2D=rho*Ca*pi*(HEIGHT*0.5*10^(-3))^4*CpYZ*CpXY*CpXZ;
end
At=LENGTH*A2D*10^-3;
A(4,4)=At*lambda;

% Pitch
B=LENGTH/HEIGHT;
Ca=ppval(CA2DR,B);
if(B>=1)
    A2D=rho*Ca*pi*(LENGTH*0.5*10^(-3))^4*CpYZ*CpXY*CpXZ;
else
    A2D=rho*Ca*pi*(HEIGHT*0.5*10^(-3))^4*CpYZ*CpXY*CpXZ;
end
At=WIDTH*10^-3*A2D;
A(5,5)=At*lambda;

%Yaw
B=WIDTH/LENGTH;
Ca=ppval(CA2DR,B);
if(B>=1)
    A2D=rho*Ca*pi*(WIDTH*0.5*10^(-3))^4*CpYZ*CpXY*CpXZ;
else
    A2D=rho*Ca*pi*(LENGTH*0.5*10^(-3))^4*CpYZ*CpXY*CpXZ;
end
At=A2D*HEIGHT*10^-3;
A(6,6)=At*lambda;

end



