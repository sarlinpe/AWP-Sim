clear all; close all;

l1 = 2;
l2 = 1;

y_zero = @(t1,t2) l1*sin(t1) + l2*sin(t1+t2);

fimplicit(y_zero,[0, pi, -pi, pi], 'k');

xlabel('\theta_1')
xticks(0:pi/4:pi)
xticklabels({'0','\pi/4','\pi/2','3\pi/4','\pi'})

ylabel('\theta_2')
yticks([-pi -pi/2 0 pi/2 pi])
yticklabels({'-\pi', '-\pi/2', 0','\pi/2','\pi'})