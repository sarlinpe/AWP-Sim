function circlemaking(xcenter,ycenter,rcircle)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step
ang_rot=0:0.01:2*pi;
xp_center=rcircle*cos(ang_rot);
yp_center=rcircle*sin(ang_rot);
plot(xcenter+xp_center,ycenter+yp_center,'k','LineWidth',1.5);
end