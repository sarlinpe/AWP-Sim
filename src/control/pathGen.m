%% animation specs
d=ceil(length(T)/10); %frame speed
j=1:d:length(T); 
%% showing path including obstacles
figure
for i=1:length(j)-1
hold on
plot([x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'go',[0 x1(j(i))],[0 y1(j(i))],'g',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'g')
hold on
plot([x1d(j(i)) x2d(j(i))],[y1d(j(i)) y2d(j(i))],'ko',[0 x1d(j(i))],[0 y1d(j(i))],'k',[x1d(j(i)) x2d(j(i))],[y1d(j(i)) y2d(j(i))],'k')
hold on
filledCircle([5,10],0.5,1000,'k');
hold on 
filledCircle([-10,5],1.2,1000,'k');
hold on 
filledCircle([-2,13],0.8,1000,'k');
hold on 
filledCircle([-5.2,8],0.8,1000,'k');
hold on 
filledCircle([9,4.5],1,1000,'k');
    
    xlabel('x [m]','Interpreter','Latex')
    ylabel('y [m]','Interpreter','Latex')
    axis([-L1-L2 L1+L2 0 L1+L2]);
    grid
    set(gcf,'position',[200,200,700,350]);
 plotopt
 pathgen = ['pathgen_' testcase '_Kp1_' num2str(Kp1) '_Kd1_' num2str(Kd1) '_Ki1_' num2str(Ki2) '_Kp2_' num2str(Kp2) '_Kd2_' num2str(Kd2) '_Ki2_' num2str(Ki2) ];
    
end
hold on
plot([x1(length(T)) x2(length(T))],[y1(length(T)) y2(length(T))],'bo',[0 x1(length(T))],[0 y1(length(T))],'b',[x1(length(T)) x2(length(T))],[y1(length(T)) y2(length(T))],'b')
hold on
plot([x1(1) x2(1)],[y1(1) y2(1)],'ro',[0 x1(1)],[0 y1(1)],'r',[x1(1) x2(1)],[y1(1) y2(1)],'r')
hold on
plot(x2,y2,'g--','LineWidth',1.1)
plot(x2d,y2d,'k--','LineWidth',1.1)
print(gcf,'-dpng',pathgen,'-r400')
%drawnow;



