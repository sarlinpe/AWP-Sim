function [ th ] = inverseKin( robot, xy )

x = xy(1);
y = xy(2);
l1 = robot.l1;
l2 = robot.l2;

% Two sets of solutions
th = NaN(2,2);

c2 = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);
if (c2 > 1) || (c2 < -1)
    return
end
c2 = [c2,c2];
s2 = [1,-1] .* sqrt(1 - c2.^2);

s1 = ((l1+l2*c2)*y - l2*s2*x)/(x^2 + y^2);
c1 = ((l1+l2*c2)*x + l2*s2*y)/(x^2 + y^2);

th1 = atan2(s1,c1);
th2 = atan2(s2,c2);
th = [th1', th2'];

end

