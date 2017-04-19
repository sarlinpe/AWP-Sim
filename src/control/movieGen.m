%% animation specs
d=30; %frame speed
j=1:d:length(T); 
%% showing path including obstacles
figure
for i=1:length(j)-1
hold off
plot([x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'ko',[0 x1(j(i))],[0 y1(j(i))],'k',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'k')
hold on
plot([x1d(j(i)) x2d(j(i))],[y1d(j(i)) y2d(j(i))],'bo',[0 x1d(j(i))],[0 y1d(j(i))],'b',[x1d(j(i)) x2d(j(i))],[y1d(j(i)) y2d(j(i))],'b')
hold on
filledCircle([5,10],0.5,1000,'k')
hold on 
filledCircle([-10,5],1.2,1000,'k')
hold on 
filledCircle([-2,13],0.8,1000,'k')
hold on 
filledCircle([-5.2,8],0.8,1000,'k')
hold on 
filledCircle([9,4.5],1,1000,'k')
    title('Workspace, blue=desired, black=controlled')
    xlabel('x')
    ylabel('y')
    %hl = legend('Controlled Robot','','','','Reference Robot');
    %set(hl,'Interpreter','Latex','Orientation','Vertical','FontSize',16);
    axis([-L1-L2 L1+L2 0 L1+L2]);
    grid
    set(gcf,'position',[200,200,700,350])
    hold on
    MM(i)=getframe(gcf);
end
drawnow;
