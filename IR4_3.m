clc
clear

% Desired end-effector HTMs
xe=[0.2 0.1 0.3 deg2rad([10 30 20])];
Tse{1}=[eul2rotm([xe(6) xe(5) xe(4)]) [xe(1:3)]';0 0 0 1];
Tse{2}=[0 -1 0 0.2; 1 0 0 0.3; 0 0 1 0.2; 0 0 0 1];

% Rotation axes by column
omg_hat=[[0 0 1]' [0 1 0]' [0 1 0]' [1 0 0]' [0 1 0]' [1 0 0]'];

% Joint home positions by column
q=[[0 0 0.15]' [0 0 0.25]' [0.25 0 0.25]' [0.45 0 0.25]' [0.45 0 0.25]' ...
    [0.45 0 0.25]'];

% Screw axes by column and home configurations by cell
for i=1:6
    S(1:6,i)=[omg_hat(:,i); -cross(omg_hat(:,i),q(:,i))];
    M{i}=[roty(pi/2) q(:,i); 0 0 0 1];
end
M{1}=[eye(3) q(:,1); 0 0 0 1];

% Inverse kinematics
% Solution storage
theta_list=[];
% Acceptable error
epsilon=0.001;
for i=1:size(Tse,2)
    % Resetting the angles and errors
    theta=zeros(6,1);
    omg_s=ones(3,1);
    v_s=ones(3,1);
    % While the angular or linear difference is above acceptable error
    while norm(omg_s)>epsilon || norm(v_s)>epsilon
        % Current end-effector
        Tsb=FKinSpace(M{end},S,theta);
        % Space twist
        Vs=Adjoint(Tsb)*se3ToVec(MatrixLog6(TransInv(Tsb)*Tse{i}));
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
x_vec1=[0 0 0]';
x_vec2=[0 0 0]';
for j=1:size(theta_list,1)
    temp1=FKinSpace(M{j},S(:,1:j),theta_list(1:j,1));
    temp2=FKinSpace(M{j},S(:,1:j),theta_list(1:j,end));
    x_vec1(1:3,j+1)=temp1(1:3,4);
    x_vec2(1:3,j+1)=temp2(1:3,4);
end

% Plotting
hold on
trplot(eye(4),'frame','{s}','color','k','length',0.05)
for j=1:size(theta_list,2)
    trplot(FKinSpace(M{end},S,theta_list(:,j)),'length',0.05,'color','r')
end
plot3(x_vec1(1,:),x_vec1(2,:),x_vec1(3,:),'b-o')
plot3(x_vec2(1,:),x_vec2(2,:),x_vec2(3,:),'g-o')
grid on
axis equal
xlabel("X")
ylabel("Y")
zlabel("Z")