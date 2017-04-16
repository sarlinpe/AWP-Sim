function [ h ] = drawRobot( wspace, robot, th1th2, s )

if ( nargin < 4 )
    s = 'r';
end

[xy2, xy1] = forwardKin(robot, th1th2);
xy0 = [0,0];

hs1 = plot(wspace, [xy0(1),xy1(1)], [xy0(2),xy1(2)], s, 'Linewidth', 1);
hs2 = plot(wspace, [xy1(1),xy2(1)], [xy1(2),xy2(2)], s, 'Linewidth', 1);

effector.center = xy2;
effector.radius = 0.05;
hc2 = drawCircle(wspace, effector, 'k');

joint.center = xy1;
joint.radius = 0.05;
hc1 = drawCircle(wspace, joint, 'k');

base.center = xy0;
base.radius = 0.15;

t = linspace(0,pi,100);
drawxy = base.center + base.radius * [cos(t'),sin(t')];
fill(wspace, drawxy(:,1),drawxy(:,2),[1,1,1],'FaceColor','k','EdgeColor','none');

h = [hs1, hs2, hc1, hc2];

end

