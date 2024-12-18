clc
clear

% Angles and angular velocity
theta=deg2rad([50 10 -15 30 30 -15]');
theta_dot=deg2rad([10 80 -50 15 20 -50]');

% Rotation axes by column
omg_hat=[[0 0 1]' [0 1 0]' [0 1 0]' [1 0 0]' [0 1 0]' [1 0 0]'];

% Joint home positions by column
q=[[0 0 0.15]' [0 0 0.25]' [0.25 0 0.25]' [0.45 0 0.25]' [0.45 0 0.25]' ...
    [0.45 0 0.25]'];

% Screw axes by column
for i=1:size(theta,1)
    S(1:6,i)=[omg_hat(:,i); -cross(omg_hat(:,i),q(:,i))];
end

% End-effector home configuration HTM
M=[roty(pi/2) q(:,6); 0 0 0 1];

% Matrix exponential HTMs
E{1}=MatrixExp6(VecTose3(S(:,1)*theta(1)));
for i=2:size(theta,1)
    E{i}=E{i-1}*MatrixExp6(VecTose3(S(:,i)*theta(i)));
end

% Space Jacobian
Js=[S(:,1)];
for i=2:size(theta,1)
    Js(:,i)=Adjoint(E{i-1})*S(:,i);
end

% End-effector body twist
Vb=Adjoint(TransInv(E{end}))*Js*theta_dot;

% End-effector linear velocity in {s}
Ts6=FKinSpace(M,S,theta);
v6=Ts6(1:3,1:3)*Vb(4:6)

% Plotting
hold on
for i=1:size(theta,1)
    if [Js(1:3,i)]~=[0 0 0]
        arrow3([0 0 0],[Js(1:3,i)]','g')
    end
    if [Js(4:6,i)]~=[0 0 0]
        arrow3([0 0 0],[Js(4:6,i)]','b')
    end
end
arrow3([0 0 0],v6','r')
axis equal
axis([-0.7 0.7 -0.5 1 -0.4 0.5])
grid on
xlabel("X")
ylabel("Y")
zlabel("Z")
title("Space Jacobian and End-effector Velocity Vectors")
arrow3("update")
