clc
clear

% Angles
theta=deg2rad([40 20 30]);

% Screw axes by column
S=[[0 0 1 0 0 0]' [0 0 1 0 -0.3 0]' [0 0 1 0 -0.5 0]'];

% Screw axes 1 and 2 matrix components
omg_m=[0 -1 0; 1 0 0; 0 0 0];
S1_v=S(4:6,1);
S2_v=S(4:6,2);

% Rotation matrix
Rs1=eye(3)+sin(theta(1))*omg_m+(1-cos(theta(1)))*omg_m^2;
R12=eye(3)+sin(theta(2))*omg_m+(1-cos(theta(2)))*omg_m^2;

% PoE HTMs
Es1=[Rs1 (eye(3)*theta(1)+(1-cos(theta(1)))*omg_m+(theta(1)-sin(theta(1))) ...
    *omg_m^2)*S1_v; 0 0 0 1];
E12=[R12 (eye(3)*theta(2)+(1-cos(theta(2)))*omg_m+(theta(2)-sin(theta(2))) ...
    *omg_m^2)*S2_v; 0 0 0 1];
Es2=Es1*E12;
Rs2=Es2(1:3,1:3);

% Linear velocity skew-symmetric matrices
vs1=Es1(1:3,4);
vs1_m=[0 -vs1(3) vs1(2); vs1(3) 0 -vs1(1); -vs1(2) vs1(1) 0];
vs2=Es2(1:3,4);
vs2_m=[0 -vs2(3) vs2(2); vs2(3) 0 -vs2(1); -vs2(2) vs2(1) 0];

% Adjoints
Es1_adj=[Rs1 zeros(3); vs1_m*Rs1 Rs1];
Es2_adj=[Rs2 zeros(3); vs2_m*Rs2 Rs2];

% Jacobian
Js=[S(:,1) Es1_adj*S(:,2) Es2_adj*S(:,3)]

