function [ collision ] = obsCollision( obs, xy1, xy2 )

n = 20;
p = size(xy1, 1);
xy0 = zeros(p,2);

seg1x = linspacen(xy0(:,1),xy1(:,1),n);
seg1y = linspacen(xy0(:,2),xy1(:,2),n);
seg2x = linspacen(xy1(:,1),xy2(:,1),n);
seg2y = linspacen(xy1(:,2),xy2(:,2),n);

segx = [seg1x, seg2x];
segy = [seg1y, seg2y];

dist2 = (segx - obs.center(1)).^2 + (segy - obs.center(2)).^2;
collision = any(dist2 < obs.radius^2, 2);

% for i = 1:p
%     seg1 = [linspace(0,xy1(i,1),n)',linspace(0,xy1(i,2),n)'];
%     seg2 = [linspace(xy1(i,1),xy2(i,1),n)',linspace(xy1(i,2),xy2(i,2),n)'];
%     
%     seg = [seg1;seg2];
%     dist2 = sum((seg - obs.center).^2,2);
%     collision(i) = any(dist2 < obs.radius^2);
% end

end