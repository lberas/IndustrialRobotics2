clc
clear

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

% Iterating angles and saving end-effector xyz position
x=[];
y=[];
z=[];
i=0;
for theta1=-pi/2:pi/12:pi/2
    E1=MatrixExp6(VecTose3(S(:,1))*theta1);
    for theta2=deg2rad(-70):pi/12:pi/2
        E2=MatrixExp6(VecTose3(S(:,2))*theta2);
        E12=E1*E2;
        for theta3=-pi/2:pi/12:pi/2
            E3=MatrixExp6(VecTose3(S(:,3))*theta3);
            i=i+1;
            Ts6=E12*E3*M{end};
            x(i)=Ts6(1,4);
            y(i)=Ts6(2,4);
            z(i)=Ts6(3,4);
        end
    end
end

% Gathering joint positions for plotting
theta_list=zeros(6,1);
for j=1:size(S,2)
    temp1=FKinSpace(M{j},S(:,1:j),theta_list(1:j,1));
    x_vec1(1:3,j)=temp1(1:3,4);
end
temp1=FKinSpace(M{end},S,theta_list(:,1));
x_vec1=[[0 0 0]' x_vec1 temp1(1:3,4)];

% Plotting
hold on
plot3(x,y,z,'b.')
plot3(x_vec1(1,:),x_vec1(2,:),x_vec1(3,:),'r-o')
axis equal
axis([-0.2 0.5 -0.5 0.5 -0.2 0.7])
grid on
xlabel("X")
ylabel("Y")
zlabel("Z")
title("Workspace")
