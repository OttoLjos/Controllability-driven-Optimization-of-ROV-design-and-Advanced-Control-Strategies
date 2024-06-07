% m = 13.5;
% dim = [0.378 0.575 0.457];
% [I, COG] = InertiaDyadic(3,m,dim);
% COG = [0 0 0.03];
% Gram_sys = CreateROVLin(m, I, COG, dim);

function Gram_sys = CreateROVLin(m, I, COB, dim, reduced)

BlueROVHeavy = 0;

ROV = initializeModelMatrices(m, I, COB, dim, BlueROVHeavy);
% calcAcceleration(ROV.M,ROV.C,ROV.D,ROV.G);
% calcNED()

% Gram_sys.v0 = [[0 0 0 0 0 0]',[0.2 0 0 0 0 0]',[0 0.2 0 0 0 0.1]',[0 0 0.2 0 0 0]',[0 0 0 0.2 0 0]',[0 0 0 0 0.2 0]'];
% Gram_sys.n0 = [[0 0 0 0 0 0]',[0 0 0 0.2 0 0]',[0 0 0 0 0.2 0]',[0 0 0 0 0 0.2]'];

Gram_sys.v0 = [[0 0 0 0 0 0]',[0.2 0 0 0 0 0]',[0 0.2 0 0 0 0.0]',[0 0 0.2 0 0 0]'];
Gram_sys.n0 = [0 0 0 0 0 0]';


[b_, number_of_v0] = size(Gram_sys.v0);
[b_, number_of_n0] = size(Gram_sys.n0);
number_of_linearizations = number_of_v0 * number_of_n0;

Gram_sys.linearizations = {};
count = 0;
for i = 1:number_of_n0
    for j = 1:number_of_v0
        count = count+1;
%         disp(count)
%         disp(Gram_sys.v0(:,j)')
%         disp(Gram_sys.n0(:,i)')
        Gram_sys.linearizations{end+1} = linearize_ROV(ROV, Gram_sys.v0(:,j), Gram_sys.n0(:,i),reduced);
    end
end

cost = 0;
Gram_sys.T = eye(8);
Gram_sys.T(7:8,7:8) = 0.001*eye(2);

% 3 9 15 21 v0 = [0 1 0 0 0 0] feiler
for i = 1:number_of_linearizations
try
%     [V,D] = eig(Gram_sys.linearizations{i}.A);
%     format short
%     disp(D)
%gramian = Gram_sys.T*gram(Gram_sys.linearizations{i},'c')*Gram_sys.T';
% disp(i)
%disp(gramian)
%eig(gramian);
%cost = cost + sum(eig(gramian));
% disp(Gram_sys.linearizations{i}.A)
% disp(eig(Gram_sys.linearizations{i}.A))
catch
%     disp(i)
end
end
%disp(cost)
Gram_sys.ROV = ROV;
% [V,D] = eig(Gram_sys.linearizations{1,1}.A);
% format short
% disp(D)
% disp(V)
% syms u v w p q r x y z phi theta pzi real
% double(subs(Gram_sys.ROV.C, [u v w p q r], Gram_sys.v0'))
% double(subs(Gram_sys.ROV.D, [u v w p q r], Gram_sys.v0'))
% double(subs(Gram_sys.ROV.J, [u v w p q r x y z phi theta pzi], [Gram_sys.v0', Gram_sys.n0']))
% double(subs(Gram_sys.ROV.J_s, [u v w p q r x y z phi theta pzi], [Gram_sys.v0', Gram_sys.n0']))
% tf(Gram_sys.linearizations{1,1})
% step(Gram_sys.linearizations{1,1})
format long
end

function linear_model = linearize_ROV(ROV_parameters, Velocity, Position, Reduced)
R = ROV_parameters;
syms u v w p q r x y z phi theta pzi real
if Reduced
    A = double(subs(R.As, ...
                        [u v w p q r x y z phi theta pzi], ...
                        [Velocity' Position']));
    B = R.Bs;    C = eye(6);    D = zeros(6);
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