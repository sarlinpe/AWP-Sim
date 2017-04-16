clear; close all;

%% Create occupancy grid

t1_lim = [0,pi];
t2_lim = [-pi, pi];
res = 0.01;

l1 = 2;
l2 = 1;
y_zero = @(t1,t2) l1*sin(t1) + l2*sin(t1+t2);

t1 = t1_lim(1):res:t1_lim(2);e
t2 = t2_lim(1):res:t2_lim(2);
[T1,T2] = meshgrid(t1,t2);

map_mat = y_zero(T1,T2) < 0;
map = robotics.BinaryOccupancyGrid(map_mat,res);
%inflate(map,1000);
figure(); map.show()

%% Find shortest path

startLocation = [2000, 10000];
endLocation = [600, 62800];

prm = robotics.PRM(map,1000);
prm.ConnectionDistance = 8000;
path = findpath(prm,startLocation,endLocation);
show(prm)

%% Generate reference inputs

v_max = [1,1];
a_max = [0.7,0.7];

[lspb, T_tot] = lspb_init(path,v_max,a_max);

