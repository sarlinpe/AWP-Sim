function h = drawCircle( ax, c, style, filled)

if ( nargin < 3 )
    style='b-';
end
if ( nargin < 4 )
    filled = false;
end
NOP = 30;

th = linspace(0,2*pi,NOP);
xy = c.center + c.radius * [cos(th'),sin(th')];

if filled
    h = fill(ax,xy(:,1),xy(:,2),[1,1,1],'FaceColor',style,'EdgeColor','none');
else
    h = plot(ax,xy(:,1),xy(:,2),style);
end

end