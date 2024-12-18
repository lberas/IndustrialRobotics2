clc
clear

% Angles
theta=deg2rad([40 20 30]);

% Angular velocities
theta_dot=[0.2 0 0];

% Screw axes by column
S=[[0 0 1 0 0 0]' [0 0 1 0 -0.3 0]' [0 0 1 0 -0.5 0]'];

% End-effector home HTM
M=[1 0 0 0.6; 0 1 0 0; 0 0 1 0; 0 0 0 1];

% PoE HTMs
Es1=MatrixExp6(VecTose3(S(:,1))*theta(1));
E12=MatrixExp6(VecTose3(S(:,2))*theta(2));
Es2=Es1*E12;
E23=MatrixExp6(VecTose3(S(:,3))*theta(3));
Es3=Es2*E23;

% Space Jacobian
Js=[S(:,1) Adjoint(Es1)*S(:,2) Adjoint(Es2)*S(:,3)];

% Space twist
Vs=Js*theta_dot';

% End-effector HTM
Tse=Es3*M;
Tes=TransInv(Tse);

% Body Twist
Vb=Adjoint(Tes)*Vs;

% End-effector velocity in space frame
ve=Tse(1:3,1:3)*Vb(4:6)

% Joint HTMs for plotting
Ts3=Es2*[1 0 0 0.5; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ts2=Es1*[1 0 0 0.3; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ts1=Es1;

% Plotting
hold on
trplot(eye(4),'length',0.1,'color','r','labels','XY ')
trplot(Ts1,'length',0.1,'labels','XY ')
trplot(Ts2,'length',0.1,'labels','XY ')
trplot(Ts3,'length',0.1,'labels','XY ')
trplot(Tse,'length',0.1,'color','k','labels','XY ')
arrow3([Tse(1:3,4)]',[Tse(1:3,4)+ve]')

axis equal
xlabel("X")
ylabel("Y")
title("End-effector velocity")
grid on