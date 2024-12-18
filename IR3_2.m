clc
clear

% Cartesian trajectory paramaters
time=4;
steps=7;
method=5;

% Starting and final configuration
T_start=[0 0 1 0.5; 0 1 0 0.1; -1 0 0 0.2; 0 0 0 1];
T_final=[1 0 0 0.1; 0 1 0 0.1; 0 0 1 0.2; 0 0 0 1];

% Rotation axes by column
omg_hat=[[0 0 1]' [0 -1 0]' [1 0 0]' [0 -1 0]' [-1 0 0]' [0 -1 0]' [-1 0 0]'];

% Joint and end-effector home positions by column
q=[[0 0 0.1]' [0.1 0 0.2]' [0.2 0 0.2]' [0.3 0 0.2]' [0.4 0 0.2]' ...
    [0.5 0 0.2]' [0.6 0 0.2]'];

% Screw axes by column and home configurations by cell
S=[0 0 1 0 0 0]';
M={[1 0 0 0; 0 1 0 0; 0 0 1 0.1; 0 0 0 1]};
for i=2:7
    S(1:6,i)=[omg_hat(:,i); -cross(omg_hat(:,i),q(:,i))];
    M{i}=[roty(pi/2) q(:,i); 0 0 0 1];
end
M{8}=[roty(pi/2) [0.7 0 0.2]'; 0 0 0 1];

% Generating path HTMs
traj=CartesianTrajectory(T_start,T_final,time,steps,method);

% Acceptable error
e=0.001;
% Finding angles for eatch path HTM
theta_list=[];
theta_old=zeros(7,1);
for i=1:max(size(traj))
    [theta,success]=IKinSpace(S,M{end},traj{i},theta_old,e,e);
    theta_list(:,i)=theta;
    theta_old=theta;
end

% Gathering joint and end-effector positions for plotting
x_vec1=[];
x_vec2=[];
x_vec3=[];
for j=1:size(theta_list,1)
    temp1=FKinSpace(M{j},S(:,1:j),theta_list(1:j,1));
    temp2=FKinSpace(M{j},S(:,1:j),theta_list(1:j,4));
    temp3=FKinSpace(M{j},S(:,1:j),theta_list(1:j,end));
    x_vec1(1:3,j)=temp1(1:3,4);
    x_vec2(1:3,j)=temp2(1:3,4);
    x_vec3(1:3,j)=temp3(1:3,4);
end
x_vec1=[[0 0 0]' x_vec1];
x_vec2=[[0 0 0]' x_vec2];
x_vec3=[[0 0 0]' x_vec3];
temp1=FKinSpace(M{end},S,theta_list(:,1));
temp2=FKinSpace(M{end},S,theta_list(:,4));
temp3=FKinSpace(M{end},S,theta_list(:,end));
x_vec1(1:3,end+1)=temp1(1:3,4);
x_vec2(1:3,end+1)=temp2(1:3,4);
x_vec3(1:3,end+1)=temp3(1:3,4);

% Plotting
hold on
trplot(eye(4),'frame','{s}','color','k','length',0.05)
for j=1:size(theta_list,2)
    trplot(FKinSpace(M{end},S,theta_list(:,j)),'length',0.05,'labels','   '...
        ,'color','r')
end
plot3(x_vec1(1,:),x_vec1(2,:),x_vec1(3,:),'b-o')
plot3(x_vec2(1,:),x_vec2(2,:),x_vec2(3,:),'c--o')
plot3(x_vec3(1,:),x_vec3(2,:),x_vec3(3,:),'g-o')
grid on
axis equal
axis([-0.1 0.55 -0.1 0.2 0 0.4])
xlabel("X")
ylabel("Y")
zlabel("Z")