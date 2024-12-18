clc
clear

% Linear end-effector velocity
ve=[-0.1 0 0.05]';

% Starting configuration
T=[0 0 1 0.5; 0 1 0 0.1; -1 0 0 0.2; 0 0 0 1];

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

% Inverse kinematics
% Solution storage
theta_list=[];
% Acceptable error
epsilon=0.001;
% Time step
dt=0.5;
i=0;
for t=0:dt:2
    i=i+1; % Counting time step iterations
    % If first time step iteration, start from home configuration
    % Else, start from the previous time step configuration
    if i==1
        theta=zeros(7,1);
    else
        theta=theta_list(:,i-1);
    end
    % Desired end-effector HTM
    Tsd=T+[zeros(3) ve*t; 0 0 0 0];
    % Resetting the errors
    omg_s=ones(3,1);
    v_s=ones(3,1);
    % While the angular or linear difference is above acceptable error
    while norm(omg_s)>epsilon || norm(v_s)>epsilon
        % Current end-effector
        Tsb=FKinSpace(M{end},S,theta);
        % Space twist
        Vs=Adjoint(Tsb)*se3ToVec(MatrixLog6(TransInv(Tsb)*Tsd));
        % Space Jacobian
        Js=JacobianSpace(S,theta);
        % Updated theta
        theta=theta+pinv(Js)*Vs;
        omg_s=Vs(1:3);
        v_s=Vs(4:6);
    end
    theta_list(:,i)=theta;
end

% Gathering joint and end-effector positions for plotting
x_vec1=[];
x_vec2=[];
for j=1:7
    temp1=FKinSpace(M{j},S(:,1:j),theta_list(1:j,1));
    temp2=FKinSpace(M{j},S(:,1:j),theta_list(1:j,end));
    x_vec1(1:3,j)=temp1(1:3,4);
    x_vec2(1:3,j)=temp2(1:3,4);
end
x_vec1=[[0 0 0]' x_vec1];
x_vec2=[[0 0 0]' x_vec2];
temp1=FKinSpace(M{end},S,theta_list(:,1));
temp2=FKinSpace(M{end},S,theta_list(:,end));
x_vec1(1:3,end+1)=temp1(1:3,4);
x_vec2(1:3,end+1)=temp2(1:3,4);

% Plotting
hold on
trplot(eye(4),'frame','{s}','color','k','length',0.05)
for j=1:size(theta_list,2)
    trplot(FKinSpace(M{end},S,theta_list(:,j)),'length',0.05,'labels','   '...
        ,'color','r')
end
plot3(x_vec1(1,:),x_vec1(2,:),x_vec1(3,:),'b-o')
plot3(x_vec2(1,:),x_vec2(2,:),x_vec2(3,:),'g-o')
grid on
axis equal
axis([-0.1 0.55 -0.1 0.2 0 0.4])
xlabel("X")
ylabel("Y")
zlabel("Z")