clc
clear

hold on

% Angles
theta1=deg2rad(40);
theta2=deg2rad(20);

% Plotting end-effector HTM every 5 degrees
for theta3=-90:5:90
    theta3=deg2rad(theta3);
    i=[theta1+theta2+theta3
        0.3*cos(theta1)+0.2*cos(theta1+theta2)+0.1*cos(theta1+theta2+theta3)
        0.3*sin(theta1)+0.2*sin(theta1+theta2)+0.1*sin(theta1+theta2+theta3)];
    T=[rot2(theta1+theta2+theta3) i(2:3); 0 0 0];
    trplot2(T, 'length', 0.01,'labels', 'X ')
end

% Plotting parameters
axis equal
xlabel('X')
ylabel('Y')
title('3R Geometric Workspace')