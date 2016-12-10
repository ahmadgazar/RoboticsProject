syms m1 m2 m3 m4
syms l1 l2 l3 l4
syms I1zz I2zz I3zz I4zz
syms q1 q2 q3 q4
syms d1 d2 d3 d4
syms q1d q2d q3d q4d
syms q1dd q2dd q3dd q4dd
syms g0
syms theta1 theta2 theta3 theta4
syms K1 K2 K3 K4

% m1 = 10;
% m2 = 7;
% m3 = 7;
% m4 = 5;

% l1 = 1;
% l2 = 0.7;
% l3 = 0.7; 
% l4 = 0.5;

% d1 = l1/2;
% d2 = l2/2;
% d3 = l3/2; 
% d4 = l4/2;

% g0 = -9.81;

%I1zz = 0.9333;
%I2zz = 0.6533;
%I3zz = 0.6533;
%I4zz = 	;

%% Positions of Center of masses of the links (pci)
pc1 = [d1*cos(q1); d1*sin(q1)];
pc2 = [l1*cos(q1) + d2*cos(q1+q2); l1*sin(q1)+d2*sin(q1+q2)];
pc3 = [l1*cos(q1) + l2*cos(q1+q2) + d3*cos(q1+q2+q3); l1*sin(q1) + l2*sin(q1+q2) + d3*sin(q1+q2+q3)];
pc4 = [l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3) + d4*cos(q1+q2+q3+q4); l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3) + d4*sin(q1+q2+q3+q4)];

%% Velocities of Center of masses of the links (vci)
vc1 = [-d1*sin(q1)*q1d; d1*cos(q1)*q1d];
vc1Norm = vc1.'* vc1;

vc2 = [(-d2*sin(q1+q2)-l1*sin(q1))*q1d-(d2*sin(q1+q2)*q2d); (d2*cos(q1+q2)+l1*cos(q1))*q2d-(d2*cos(q1+q2)*q2d)];
vc2Norm = vc2.'* vc2;

vc3 = [(-l2*sin(q1+q2)-l1*sin(q1)-d3*sin(q1+q2+q3))*q1d+(-l2*sin(q1+q2)-d3*sin(q1+q2+q3))*q2d-d3*sin(q1+q2+q3)*q3d; (l2*cos(q1+q2)+l1*cos(q1)+d3*cos(q1+q2+q3))*q1d+(l2*cos(q1+q2)+d3*cos(q1+q2+q3))*q2d+d3*cos(q1+q2+q3)*q3d];
vc3Norm = vc3.'* vc3;

vc4 = [(-d4*sin(q1+q2+q3+q4)-l2*sin(q1+q2)-l1*sin(q1)-l3*sin(q1+q2+q3))*q1d+(-d4*sin(q1+q2+q3+q4)-l2*sin(q1+q2)-l3*sin(q1+q2+q3))*q2d+(-d4*sin(q1+q2+q3+q4)-l3*sin(q1+q2+q3))*q3d+(-d4*sin(q1+q2+q3+q4)*q4d); (d4*cos(q1+q2+q3+q4)+l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3))*q1d+(d4*cos(q1+q2+q3+q4)+l2*cos(q1+q2)+l3*cos(q1+q2+q3))*q2d+(d4*cos(q1+q2+q3+q4)+l3*cos(q1+q2+q3))*q3d+d4*cos(q1+q2+q3+q4)*q4d];
vc4Norm = vc4.'* vc4;


%% Kinetic Energy of link 1
T1ang = 0.5*I1zz*q1d^2;
T1lin = 0.5*m1*vc1Norm;
T1 = T1ang + T1lin;

%% Kinetic Energy of link 2
T2ang = 0.5*I2zz*(q1d+q2d)^2;
T2lin = 0.5*m2*vc2Norm;
T2 = T2ang + T2lin;

%% Kinetic Energy of link 3
T3ang = 0.5*I3zz*(q1d+q2d+q3d)^2;
T3lin = 0.5*m3*vc3Norm;
T3 = T3ang + T3lin;

%% Kinetic Energy of link 4
T4ang = 0.5*I4zz*(q1d+q2d+q3d+q4d)^2;
T4lin = 0.5*m4*vc4Norm;
T4 = T4ang + T4lin;

%% Total Kinetic Energy

disp('***robot kinetic energy***')
T = T1+T2+T3+T4;
T= simplify(T);
T= collect(T,q1d^2);
T= collect(T,q2d^2);
T= collect(T,q3d^2);
T= collect(T,q4d^2);
%% Inertia Matrix (M)

M(1,1) = diff(T,q1d,2);
Temp1 = diff(T,q1d);
M(1,2) = diff(Temp1,q2d);
M(2,1) = M(1,2);
M(1,3) = diff(Temp1,q3d);
M(3,1) = M(1,3);
M(1,4) = diff(Temp1,q4d);
M(4,1) = M(1,4);

M(2,2) = diff(T,q2d,2);
Temp2 = diff(T,q2d);
M(2,3) = diff(Temp2,q3d);
M(3,2) = M(2,3);
M(2,4) = diff(Temp2,q4d);
M(4,2) = M(2,4);
M(1,4) = diff(Temp1,q4d);
M(4,1) = M(1,4);

M(3,3) = diff(T,q3d,2);
Temp3 = diff(T,q3d);
M(3,4) = diff(Temp3,q4d);
M(4,3) = M(3,4);

M(4,4) = diff(T,q4d,2);

disp('***robot inertia matrix***')
M = simplify(M);
Inverse = inv(M);

%% Inertia Matrix Derivative (Mdot)

Mdot = zeros(4,4);
Mdot =  sym(Mdot);


for col = 1:4
    for row = 1:4
        Mdot(row,col) = (diff(M(row,col),q1))*q1d + (diff(M(row,col),q2))*q2d + (diff(M(row,col),q3))*q3d + (diff(M(row,col),q4))*q4d;
    end
end 



%% Christoffel Matrix (C)

disp('*Christoffel matrices*')
q = [q1;q2;q3;q4];
M1 = M(:,1);
C1 = (1/2)*(jacobian(M1,q)+(jacobian(M1,q).')-diff(M,q1));
M2 = M(:,2);
C2 = (1/2)*(jacobian(M2,q)+(jacobian(M2,q).')-diff(M,q2));
M3 = M(:,3);
C3 = (1/2)*(jacobian(M3,q)+(jacobian(M3,q).')-diff(M,q3));
M4 = M(:,4);
C4 = (1/2)*(jacobian(M4,q)+(jacobian(M4,q).')-diff(M,q3));


disp('***robot Coriolis and centrifugal terms***')
qd = [q1d;q2d;q3d;q4d];
c1 = qd.'*C1*qd;
c2 = qd.'*C2*qd;
c3 = qd.'*C3*qd;
c4 = qd.'*C3*qd;
C = [c1;c2;c3;c4];

%%  Christoffel Matrix derivative (Cdot)

C1dot = zeros(4,1);
C1dot =  sym(C1dot);
C2dot = zeros(4,1);
C2dot =  sym(C2dot);

for row = 1:4
        C1dot(row,1) = (diff(C(row,1),q1))*q1d + (diff(C(row,1),q2))*q2d + (diff(C(row,1),q3))*q3d + (diff(C(row,1),q4))*q4d;
        C2dot(row,1) = (diff(C(row,1),q1d))*q1dd + (diff(C(row,1),q2d))*q2dd + (diff(C(row,1),q3d))*q3dd + (diff(C(row,1),q4d))*q4dd;

end 
Cdot = C1dot+C2dot;
%% Gravity Matrix (G)

g = [-g0;0];
U1 = -m1*g.'*pc1;
U2 = -m2*g.'*pc2;
U3 = -m3*g.'*pc3;
U4 = -m4*g.'*pc4;

disp('***robot potential energy (due to gravity)***')
U = simplify(U1+U2+U3+U4);

disp('***robot gravity term***')
G = jacobian(U,q).';
G = simplify(G);

%% Gravity Matrix Derivative (dgi/dqi)*qi
qdd = [q1dd;q2dd;q3dd;q4dd];
theta = [theta1;theta2;theta3;theta4];
k = [K1 K2 K3 K4];
K = diag(k);

%Calculating Gdot
DgDq = zeros(4,4);
DgDq = sym(DgDq);
DgDq = jacobian(G,q);
DgDq = simplify(DgDq);
Gdot = zeros(4,1);
Gdot = sym(Gdot);
Gdot = DgDq*qd;

% calculating Gddot 
D2gDq1Dqj = zeros(4,4);
D2gDq1Dqj = sym(D2gDq1Dqj);

D2gDq2Dqj = zeros(4,4);
D2gDq2Dqj = sym(D2gDq2Dqj);

D2gDq3Dqj = zeros(4,4);
D2gDq3Dqj = sym(D2gDq3Dqj);

D2gDq4Dqj = zeros(4,4);
D2gDq4Dqj = sym(D2gDq4Dqj);

D2gDq1Dqj = simplify(jacobian(DgDq(:,1),q));
D2gDq2Dqj = simplify(jacobian(DgDq(:,2),q));
D2gDq3Dqj = simplify(jacobian(DgDq(:,3),q));
D2gDq4Dqj = simplify(jacobian(DgDq(:,4),q));

Gddot1 = simplify(DgDq*qdd);
Gddot2 = simplify(D2gDq1Dqj*qd*q1d) + simplify(D2gDq2Dqj*qd*q2d) + simplify(D2gDq3Dqj*qd*q3d )+ simplify(D2gDq4Dqj*qd*q4d);
Gddot = simplify(Gddot1 + Gddot2);


%% Preprocessing
   
% MWS = get_param('Scratch1','ModelWorkspace');
MWS = get_param('Scratch3','ModelWorkspace');

%% Motor Inertias
MWS.assignin('Jm1',0.9333);
MWS.assignin('Jm2',0.6533);
MWS.assignin('Jm3',0.6533);
MWS.assignin('Jm4',0.4667);

%% Springs stiffness (Nm/rad)
MWS.assignin('k1',100);
MWS.assignin('k2',100);
MWS.assignin('k3',100);
MWS.assignin('k4',100);

%% Damping gains
MWS.assignin('Fv1',15); 
MWS.assignin('Fv2',12); 
MWS.assignin('Fv3',1); 
MWS.assignin('Fv4',1); 
%% Proportional gains
MWS.assignin('Kp1',15); 
MWS.assignin('Kp2',20); 
MWS.assignin('Kp3',5); 
MWS.assignin('Kp4',5); 

%% Derivative gains
MWS.assignin('Kd1',5); 
MWS.assignin('Kd2',10); 
MWS.assignin('Kd3',2); 
MWS.assignin('Kd4',2); 







