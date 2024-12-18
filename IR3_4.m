clc
clear

Tf=1;
t_star=0.5;
dt=Tf/98;

a=1/(t_star*(Tf-t_star));

s_dot_list=0;
s=0;
for t=0+dt:dt:Tf
    if t<t_star
        s_dot=a*t;
    elseif t>(Tf-t_star)
        s_dot=a*(Tf-t);
    else
        s_dot=a*t_star;
    end
    s_dot_list(end+1)=s_dot;
    s(end+1)=s(end)+s_dot_list(end)*dt;
end

figure(1)
plot(0:dt:Tf, s_dot_list)
xlabel('Time (s)')
ylabel('s derivative')
title('s derivative vs Time, t1=t2='+string(t_star)+' s')

figure(2)
plot(0:dt:Tf, s)
xlabel('Time (s)')
ylabel('s')
title('s vs Time, t1=t2='+string(t_star)+' s')