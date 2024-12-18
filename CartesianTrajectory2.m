function traj = CartesianTrajectory2(Xstart,Xend,Tf,N)

% Original
timegap = Tf / (N - 1);
traj = cell(1, N);
[Rstart, pstart] = TransToRp(Xstart);
[Rend, pend] = TransToRp(Xend);

% New
s_dot_list=0;
s=0;
a=4/(Tf^2);

for i = 1: N
    % New
    t=timegap*i;
    if t<(Tf/2)
        s_dot=a*t;
    else
        s_dot=a*(Tf-t);
    end
    s_dot_list(i+1)=s_dot;
    s(i+1)=s(i)+s_dot_list(i)*timegap;
    % Original
    traj{i} ...
    = [Rstart * MatrixExp3(MatrixLog3(Rstart' * Rend) * s(i)), ...
       pstart + s(i) * (pend - pstart); 0, 0, 0, 1];
end