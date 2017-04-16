function [ collision ] = groundCollision( robot, th1th2 )

th1 = th1th2(:,1);
th2 = th1th2(:,2);

collision = (robot.l1*sin(th1) + robot.l2*sin(th1+th2)) < 0;

end

