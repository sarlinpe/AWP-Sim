
robot.l1 = 1;
robot.l2 = 1;


figure();
h = axes();
daspect([1,1,1]);
hold on;

xy = [0.2,1];
plot(xy(1),xy(2),'k*');

th = inverseKin(robot, xy);

if all(isnan(th(:)))
    error('Out of workspace');
end

drawRobot(h, robot, th(1,:),'r');
if any(th(1,:) ~= th(2,:))
    drawRobot(h, robot, th(2,:),'b');
end