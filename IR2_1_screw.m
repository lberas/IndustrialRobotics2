clc
clear

hold on

% End-effector in home configuration
M=[1 0 0 0.6; 0 1 0 0; 0 0 1 0; 0 0 0 1];

% Screw axes
S1=[0 0 1 0 0 0]';
S2=[0 0 1 0 -0.3 0]';
S3=[0 0 1 0 -0.5 0]';

% Angles
theta1=deg2rad(40);
theta2=deg2rad(20);

% Plotting end-effector HTM every 5 degrees
Ts2=MatrixExp6(VecTose3(S1)*theta1)*MatrixExp6(VecTose3(S2)*theta2);
for theta3=-90:5:90
    T=Ts2*MatrixExp6(VecTose3(S3)*deg2rad(theta3))*M;
    trplot(T, 'length', 0.01,'labels', 'X  ')
end

% Plotting parameters
axis equal
xlabel('X')
ylabel('Y')
title('3R Screw Theory Workspace')