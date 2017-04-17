clear; close all;
addpath('./path_finder', './trajectory_generator');
warning('off', 'robotics:robotalgs:occgridcommon:RoundoffWarning');

%% Parameters

% Robot
robot.l1 = 1.9;
robot.l2 = 1.1;
th_init = [pi/3, -pi/4];
v_max = [0.5,0.5];
a_max = [0.2,0.2];

% Environment
th1_lim = [0,pi];
th2_lim = [-pi, pi];
x_lim = (robot.l1+robot.l2)*[-1.1,1.1];
y_lim = (robot.l1+robot.l2)*[-0.1,1.1];

obstacles = [createCircle([0.8,2.1],0.2), ...
             createCircle([-2.2,1.2],0.2), ...
             createCircle([-0.9,2.2],0.2)];

% Path finding
resolution = 100;
inflate_rad = 0.05;
nb_prm_nodes = 1500;
prm_dist = 0.5;

% GUI
display_reachable_ws = false;
show_prm = false;

%% Create UI

[cspace, wspace] = initGUI(x_lim, y_lim, th1_lim, th2_lim);

h = drawRobot(wspace, robot, th_init);
h = [h,plot(cspace, th_init(1), th_init(2), 'ro')];

for obs = obstacles
    drawCircle(wspace, obs, 'k', true);
end

%% Cspace collision map

% Build
map_cs = robotics.BinaryOccupancyGrid(diff(th1_lim),diff(th2_lim),resolution);
map_cs.GridLocationInWorld = [th1_lim(1),th2_lim(1)];
collision_cs = cspaceCollision(robot,map_cs,obstacles,inflate_rad);

% Display
im = imagesc(cspace, th1_lim,[th2_lim(2),th2_lim(1)],~collision_cs);
im.AlphaData = 0.5;
uistack(im,'bottom');

%% Build Wspace collision map

if display_reachable_ws
    % Build
    map_ws = robotics.BinaryOccupancyGrid(diff(x_lim),diff(y_lim),int16(resolution));
    map_ws.GridLocationInWorld = [x_lim(1),y_lim(1)];
    collision_ws = wspaceCollision(robot,map_ws,map_cs,th1_lim,th2_lim);
    
    % Display
    im = imagesc(wspace, x_lim,[y_lim(2),y_lim(1)],~collision_ws);
    im.AlphaData = 0.5;
    uistack(im,'bottom');
end

%% Find shortest path

prm = robotics.PRM(map_cs,nb_prm_nodes);
prm.ConnectionDistance = prm_dist;
th_current = th_init;


while 1
    th_start = th_current;
    delete(h);
    h = drawRobot(wspace, robot, th_current);
    h = [h,plot(cspace, th_current(1), th_current(2), 'ro')];

    % Get target point
    [x,y] = ginput(1);

    if gca == cspace % setpoint in c-space
        th_end = [x,y];
        h = [h,plot(cspace, th_end(1), th_end(2), 'go')]; drawnow;
        try
            path = findpath(prm,th_start,th_end);
        catch ME
            switch ME.identifier
                case 'robotics:robotalgs:prm:OccupiedLocation'
                    uiwait(msgbox('Invalid target configuration.','Error','error','modal'));
                    if dialogGUI() continue; else break; end
                otherwise
                    rethrow(ME);
            end
        end
        if size(path,1) == 0
            uiwait(msgbox('No valid path could be found.','Error','error','modal'));
            if dialogGUI() continue; else break; end
        end
    else % setpoint in w-space
        xy_end = [x,y];
        th_candidates = inverseKin(robot,xy_end);
        if all(isnan(th_candidates(:)))
            uiwait(msgbox('Target is out of workspace.','Error','error','modal'));
            if dialogGUI() continue; else break; end
        end
        if all(th_candidates(1,:) == th_candidates(2,:))
            th_candidates = th_candidates(1,:);
        end

        h = [h,plot(cspace, th_candidates(:,1), th_candidates(:,2), 'go')];
        drawnow

        th_accepted = [];
        paths = {};
        lengths = [];

        for i = 1:length(th_candidates)
            th = th_candidates(i,:);
            if th(1) < th1_lim(1) || th(1) > th1_lim(2) || ... 
                    th(2) < th2_lim(1) || th(2) > th2_lim(2)
                continue;
            end

            try
                path = findpath(prm,th_start,th);
            catch ME
                switch ME.identifier
                    case 'robotics:robotalgs:prm:OccupiedLocation'
                        continue
                    otherwise
                        rethrow(ME);
                end
            end
            if size(path,1) ~= 0 % a path is found!
                th_accepted = [th_accepted; th];
                paths = [paths, {path}];
                lengths = [lengths, sum(sqrt(sum(diff(path,1).^2,2)),1)];
            end
        end

        if size(th_accepted,1) == 0
            uiwait(msgbox('No valid configuration or path could be found.', 'Error','error','modal'));
            if dialogGUI() continue; else break; end
        end
        [~, idx] = min(lengths);
        th_end = th_accepted(idx,:);
        path = paths{idx};
    end

    h = [h,drawRobot(wspace, robot, th_end, 'b-')];
    h = [h,plot(cspace, th_end(1), th_end(2), 'bo')];
    h = [h,plot(cspace, path(:,1),path(:,2),'r*-')];
    drawnow

    if show_prm
        h = [h,figure()];
        show(prm);
    end
    th_current = th_end;

    %% Perform pruning

    path = prunePath(path, map_cs);
    h = [h,plot(cspace, path(:,1),path(:,2),'b*-')];

    %% Generate reference inputs

    [lspb, T_tot] = lspb_init(path,v_max,a_max);

    %% Plot trajectory

    t = linspace(0,T_tot);
    [q,v,a] = lspb_get(t, lspb);
    
    % In Cspace
    h = [h,plot(cspace, q(:,1),q(:,2),'b')];
    h = [h,plotProfiles(robot, lspb, t, q, v, a)];

    % In Wspace
    r = robotics.Rate(10);
    for i = 2:length(t)-1
        hold on;
        h_robot = drawRobot(wspace, robot, q(i,:), 'k');
        drawnow
        waitfor(r);
        delete(h_robot);
    end
    
    axes(cspace);
    while ~waitforbuttonpress end
    if dialogGUI() continue; else break; end
    
end