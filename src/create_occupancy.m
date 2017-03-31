clear all; close all;

t1_lim = [0,pi];
t2_lim = [-pi, pi];
res = 0.01;

l1 = 2;
l2 = 1;
y_zero = @(t1,t2) l1*sin(t1) + l2*sin(t1+t2);

t1 = t1_lim(1):res:t1_lim(2);
t2 = t2_lim(1):res:t2_lim(2);
[T1,T2] = meshgrid(t1,t2);

map_mat = y_zero(T1,T2) < 0;
map = robotics.BinaryOccupancyGrid(map_mat,res);
%inflate(map,1000);
figure(); map.show()

prm = robotics.PRM(map,1000);
prm.ConnectionDistance = 8000;
startLocation = [2000, 10000];
endLocation = [600, 62800];
path = findpath(prm,startLocation,endLocation);
show(prm)


