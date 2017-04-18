function [ f ] = plotProfiles(robot, lspb, t, q, v, a)

f = figure('Name', 'Trajectory profiles');

subplot(2,2,1);
title('Positions of joints');
hold on; grid on;
xlim([0, t(end)]);
plot([0 lspb.T_stamps],lspb.p(:,1),'k*-');
plot(t,q(:,1),'b','LineWidth',1);
plot([0 lspb.T_stamps],lspb.p(:,2),'k*-')
plot(t,q(:,2),'r','LineWidth',1);

subplot(2,2,2);
title('Velocities of joints');
hold on; grid on;
xlim([0, t(end)]);
plot(t,v(:,1),'b','LineWidth',1);
plot(t,v(:,2),'r','LineWidth',1);

subplot(2,2,3);
title('Accelerations of joints');
hold on; grid on;
xlim([0, t(end)]);
plot(t,a(:,1),'b','LineWidth',1);
plot(t,a(:,2),'r','LineWidth',1);

l1 = robot.l1;
l2 = robot.l2;
th1 = q(:,1);
th2 = q(:,2);
J = [-l1*sin(th1)-l2*sin(th1+th2), -l2*sin(th1+th2); ...
     l1*cos(th1)+l2*cos(th1+th2), l2*cos(th1+th2)];
C = J*[v',v'];
d = diag(C);
vxy = reshape(d,length(t),2);

subplot(2,2,4);
title('Velocities of end effector');
hold on; grid on;
xlim([0, t(end)]);
plot(t,vxy(:,1),'b','LineWidth',1);
plot(t,vxy(:,2),'r','LineWidth',1);
plot(t,sqrt(sum(vxy.^2,2)),'k','LineWidth',1);

end

