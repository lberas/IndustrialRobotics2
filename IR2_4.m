clc
clear

% Joint angles of all configurations by column
theta=[[0.4 0.4 0.4]' [0.25 0.25 0.25]' [0.025 0.025 0.025]'];

% Screw axes by column
S=[[0 0 1 0 0 0]' [0 0 1 0 -0.3 0]' [0 0 1 0 -0.5 0]'];

% End-effector home configuration HMT
M=[1 0 0 0.6; 0 1 0 0; 0 0 1 0; 0 0 0 1];

% Body Jacobians of all configurations
Jb1=JacobianBody(S,theta(:,1));
Jb2=JacobianBody(S,theta(:,2));
Jb3=JacobianBody(S,theta(:,3));

% End-effector HTM for all configurations
T1=FKinSpace(M,S,theta(:,1));
T2=FKinSpace(M,S,theta(:,2));
T3=FKinSpace(M,S,theta(:,3));

% Storing linear velocities in a matrix
v1_mat=[];
v2_mat=[];
v3_mat=[];

for a=-2*pi:pi/12:2*pi
    for b=-2*pi:pi/12:2*pi
        % Provided angular velocity equations
        thetadot=[cos(a)*sin(b) sin(a)*sin(b) cos(b)]';
        % Body twists
        Vb1=Jb1*thetadot;
        Vb2=Jb2*thetadot;
        Vb3=Jb3*thetadot;
        % End-effector linear velocities transformed to {s} and saved
        vs1=T1(1:2,1:2)*Vb1(4:5);
        v1_mat(1:2,end+1)=vs1;
        vs2=T2(1:2,1:2)*Vb2(4:5);
        v2_mat(1:2,end+1)=vs2;
        vs3=T3(1:2,1:2)*Vb3(4:5);
        v3_mat(1:2,end+1)=vs3;
    end
end

figure(1)
plot(v1_mat(1,:),v1_mat(2,:))
xlabel('x velocity')
ylabel('y velocity')
title('theta=[0.4 0.4 0.4] manipulability')

figure(2)
plot(v2_mat(1,:),v2_mat(2,:))
xlabel('x velocity')
ylabel('y velocity')
title('theta=[0.25 0.25 0.25] manipulability')

figure(3)
plot(v3_mat(1,:),v3_mat(2,:))
xlabel('x velocity')
ylabel('y velocity')
title('theta=[0.025 0.025 0.025] manipulability')