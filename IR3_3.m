clc
clear

% Time
Tf=1;
% No. of discretisation points
N=9;

% Starting and final configuration
T_start=[0 0 1 0.5; 0 1 0 0.1; -1 0 0 0.2; 0 0 0 1];
T_final=[1 0 0 0.1; 0 1 0 0.1; 0 0 1 0.2; 0 0 0 1];

traj=CartesianTrajectory2(T_start,T_final,Tf,N);

% Plotting
hold on
for j=1:N
    trplot(traj{j},'length',0.05,'labels','   ','color','r')
end
axis equal
xlabel("X")
ylabel("Y")
zlabel("Z")
grid on
axis([0.05 0.6 0.05 0.25 0.15 0.25])