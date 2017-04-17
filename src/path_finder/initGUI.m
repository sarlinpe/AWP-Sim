function [cspace, wspace] = initGUI(x_lim, y_lim, th1_lim, th2_lim)

s = get( 0,'Screensize');
w = s(3); h = s(4);
f = figure('position', [w/4, h/4, w/2, h/2]);

cspace = subplot(1,3,1);
wspace = subplot(1,3,2:3);

axes(wspace);
title('Workspace');
xlim(x_lim);
xlabel('x')
ylim(y_lim);
ylabel('y')
box on; grid on; hold on;
daspect([1,1,1]);
line(x_lim, [0,0], 'color', 'k', 'linewidth', 1); % ground

axes(cspace);
title('Configuration space');
xlim(th1_lim);
xlabel('\theta_1')
xticks(0:pi/4:pi)
xticklabels({'0','\pi/4','\pi/2','3\pi/4','\pi'})
ylim(th2_lim);
ylabel('\theta_2')
yticks([-pi -pi/2 0 pi/2 pi])
yticklabels({'-\pi', '-\pi/2', 0','\pi/2','\pi'})
box on; grid on; hold on;
daspect([1,1,1]);

colormap(f, [.7,.7,.7;1,1,1]);

end

