% *Determine jacobian*
syms theta1 theta2 theta3 t1dot t2dot t3dot L1 L2 L3 m1 m2 m3 g Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 'real'
thetaList = [theta1; theta2; theta3];
massList = [m1; m2; m3];
qdot = [t1dot; t2dot; t3dot];
%Set home matrix
%%
% |Assume that the EE coordinate frame is the identical to the base coordinate frame|
M = [1,0,0,L2+L3;
    0,1,0,0;
    0,0,1,L1;
    0,0,0,1];
%World screw axis
SList = [[0;0;1; 0;0;0], ...
        [0;-1;0; L1;0;0], ...
        [0;-1;0; L1;0;-L2;]];
%%
% *Calculate transforms wrt space*
Tworld = myFKWorld(M,SList,thetaList);
%Verify correct home position
T03 = subs(Tworld,[theta1, theta2, theta3],[0 pi/2 0]);
%%
% *Calculate jacobian*
jacobian = calcJacobian(Tworld, thetaList);

%Calculate sum of Jv and Jw's
D = zeros(3);
R01 = eye(3);
R02 = rotx(-pi/2);
R03 = rotx(-pi/2);
Rlist = {R01,R02,R03};
%Assume that mass distribution along links is symmetric
I01 = [Ixx1 0 0; 0 Iyy1 0; 0 0 Izz1];
I02 = [Ixx2 0 0; 0 Iyy2 0; 0 0 Izz2];
I03 = [Ixx3 0 0; 0 Iyy3 0; 0 0 Izz3];
Ilist = {I01;I02;I03};
for i = 1:size(thetaList)
    %calculate Jv & Jw for current joint, ie total jacobian up to current joint,
    %then fill the rest with zeros
    currentJv = jacobian(1:3,1:3);
    currentJv(1:3,1+i:end) = 0;
    currentJw = jacobian(4:6,1:3);
    currentJw(1:3,1+i:end) = 0;
    %Calculate and linearV at joint
    linearV = (massList(i)*(currentJv.')*currentJv);
    %Calculate and omega at joint
    omega = (currentJw.')*Rlist(i)*Ilist(i)*(Rlist(i).')*currentJw;
    %Calulate and sum D
    D = D + (linearV + omega);
end
K = .5*(qdot.')*D*qdot;
D = vpa(simplify(D),5);
K = vpa(simplify(K),5);

%% Deriving Potental Energy
syms Lc1 Lc2 Lc3
%Since g is only in the Z direction the G matrix can be derived easily as
Rci = [0, 0, 0;
    0, 0, 0;
    Lc1, L1+(Lc2*sin(theta2)),L1+(L2*sin(theta2))+(Lc3*sin(theta3))];
G = [0;0;g];
P = 0;
for i = 1:size(thetaList)
    P = P + massList(i,:)*G.'*Rci(:,i);
end
P = simplify(P);
gravMatrix = [diff(P,theta1);diff(P,theta2);diff(P,theta3);];

%% Deriving Coupling Matrix
C = zeros(3);
C = sym(C);
for k = 1:size(thetaList)
    for i = 1:size(thetaList)
        for j = 1:size(thetaList)
            C = C + .5*(diff(D(k,j),thetaList(i,:)) + diff(D(k,i),thetaList(j,:)) - diff(D(i,j),thetaList(k,:)))*qdot(i,:)*qdot(j,:);
        end
    end
end

%% Form "Compact" form Lagrangian
syms q1doubledot q2doubledot q3doubledot
doubleDotList = [q1doubledot;q2doubledot;q3doubledot];
Tlagrange = vpa(simplify(D*doubleDotList + C*qdot + gravMatrix),2);

symbols = [L1 L2 L3 m1 m2 m3 g Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 Lc1 Lc2 Lc3];
inputValues = [.2 .2 .2 .5 .5 .5 9.81 .1 .1 .1 .1 .1 .1 .1 .1 .1 .1 .1 .1];
Tsolved = subs(Tlagrange,symbols,inputValues);
Tsolved = simplify(Tsolved,'IgnoreAnalyticConstraints',true)

%% Functions
function J = calcJacobian(transform,thetaList)
for i = 1 : size(thetaList)
    J(:,i) = [diff(transform(1,4),thetaList(i,:));diff(transform(2,4),thetaList(i,:));diff(transform(3,4),thetaList(i,:));transform(1:3,3)];
end
end

%M is home matrix
%Slist is list of screw matricies
%thetalist is list of joint parameters
function T = myFKWorld(M, Slist, thetalist)
T = M;
for i = size(thetalist): -1: 1
    T = expTransform(Slist(:,i),thetalist(i)) * T;
end
end

%Theta in radians
%S is screw axis
function ES=expTransform(S,theta)
w = omegaToSkew(S(1:3,1));
v = S(4:6,1);
R = eye(3)+(sin(theta) * w) + ((1-cos(theta))*w^2);
V = (eye(3)*theta+(1-cos(theta))*w + (theta-sin(theta))*w^2)*v;
ES = [[R;0 0 0], [V;1]];
end

%convets omega (w) to skew matrix (w hat)
function w=omegaToSkew(omega)
w = [0, -omega(3), omega(2);
    omega(3), 0, -omega(1);
    -omega(2), omega(1), 0];
end