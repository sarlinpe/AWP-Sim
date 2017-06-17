% Title:        AWP Trajectory Planning and Control
% File:         main.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  The main simulation and GUI loop.

warning('off', 'robotics:robotalgs:occgridcommon:RoundoffWarning');

%% Model generation

robot.m1 = rho_steel*robot.l1*acr1; % mass 1
robot.mc2 = rho_steel*robot.l2*acr2; % mass 2
robot.m2 = robot.me + robot.mc2; % mass of end effector and stair 2
robot.lc1= robot.l1/2; % distance of the link 1 mass center
robot.lc2 = ((robot.l2/2)*robot.mc2+robot.l2*robot.me)/(robot.mc2+robot.me);
robot.i1 = sqrt(acr1)^4 / 12;
robot.i2 = sqrt(acr2)^4 / 12;

%% UI creation
disp('Building the graphical user interface...')

x_lim = (robot.l1+robot.l2)*[-1.1,1.1];
y_lim = (robot.l1+robot.l2)*[-0.1,1.1];
[cspace, wspace] = initGUI(x_lim, y_lim, th1_lim, th2_lim);

% Robot configuration in C-space and W-space
h = drawRobot(wspace, robot, th_init);
h = [h,plot(cspace, th_init(1), th_init(2), 'ro')];

% Obstacles
for obs = obstacles
    drawCircle(wspace, obs, 'k', true);
end

% Outer W-space limit
drawCircle(wspace, struct('center',[0,0],'radius',robot.l1+robot.l2), ...
           'k:', false, 0, pi);
drawnow

%% Cspace collision map
disp('Building the C-space collision map...')

% Build
map_cs = robotics.BinaryOccupancyGrid(diff(th1_lim),diff(th2_lim),cs_resolution);
map_cs.GridLocationInWorld = [th1_lim(1),th2_lim(1)];
collision_cs = cspaceCollision(robot,map_cs,obstacles,inflate_rad);

% Display
im = imagesc(cspace, th1_lim,[th2_lim(2),th2_lim(1)],~collision_cs);
im.AlphaData = 0.5;
uistack(im,'bottom');

%% Build Wspace collision map

if display_reachable_ws
    disp('Building the W-space collision map...')
    % Build
    ws_resolution = int16(600/diff(x_lim));
    map_ws = robotics.BinaryOccupancyGrid(diff(x_lim),diff(y_lim),ws_resolution);
    map_ws.GridLocationInWorld = [x_lim(1),y_lim(1)];
    collision_ws = wspaceCollision(robot,map_ws,map_cs,th1_lim,th2_lim);
    
    % Display
    im = imagesc(wspace, x_lim,[y_lim(2),y_lim(1)],~collision_ws);
    im.AlphaData = 0.5;
    uistack(im,'bottom');
end

%% Main loop

% Init PRM
prm = robotics.PRM(map_cs,nb_prm_nodes);
prm.ConnectionDistance = prm_dist;
th_current = th_init;


while 1
    %% Get shortest path
    th_start = th_current;
    delete(h);
    h = drawRobot(wspace, robot, th_current);
    h = [h,plot(cspace, th_current(1), th_current(2), 'ro')];

    disp('> Please select a target point in the C-space or W-space.')
    [x,y] = ginput(1); % get target point

    disp('Computing the shortest path...')
    if gca == cspace % target point in c-space
        th_end = [x,y];
        h = [h,plot(cspace, th_end(1), th_end(2), 'go')]; drawnow;
        try
            path = findpath(prm,th_start,th_end);
        catch ME
            switch ME.identifier
                case 'robotics:robotalgs:prm:OccupiedLocation'
                    uiwait(msgbox('Invalid target configuration.', ...
                        'Error','error','modal'));
                    if dialogGUI(), continue; else, break; end
                otherwise
                    rethrow(ME);
            end
        end
        if size(path,1) == 0
            uiwait(msgbox('No valid path could be found.', ...
                'Error','error','modal'));
            if dialogGUI(), continue; else, break; end
        end
    else % target point in w-space
        xy_end = [x,y];
        th_candidates = inverseKin(robot,xy_end);
        if all(isnan(th_candidates(:))) % no possible inverse
            uiwait(msgbox('Target is out of workspace.', ...
                'Error','error','modal'));
            if dialogGUI(), continue; else, break; end
        end
        if all(th_candidates(1,:) == th_candidates(2,:)) % only one inverse
            th_candidates = th_candidates(1,:);
        end

        % display all the inverse configurations
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
            catch ME % no path: inverse is in occupied space ?
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

        if size(th_accepted,1) == 0 % no valid inverse at all
            uiwait(msgbox('No valid configuration/path could be found.', ...
                'Error','error','modal'));
            if dialogGUI(), continue; else, break; end
        end
        [~, idx] = min(lengths); % accept the closest inverse
        th_end = th_accepted(idx,:);
        path = paths{idx};
    end

    if show_prm
        show(prm,'Paren',cspace,'Map','off','Path','off');
    end
    
    h = [h,drawRobot(wspace, robot, th_end, 'b-')];
    h = [h,plot(cspace, th_end(1), th_end(2), 'bo')];
    if display_waypoints
        h = [h,plot(cspace, path(:,1),path(:,2),'r*-')];
    end
    drawnow
    
    th_current = th_end;

    %% Perform pruning
    disp('Performing pruning...')
    
    path = prunePath(path, map_cs);
    if display_waypoints
        h = [h,plot(cspace, path(:,1),path(:,2),'b*-')];
    end

    %% Generate reference inputs

    [lspb_cfg, T_tot] = lspb_init(path,v_max,a_max);
    
    %% Simulate dynamic model with PID controller
    disp('Simulating the dynamic system...')
    
    [T, th_ctrl, F] = controller(robot, PID, lspb_cfg, th_start, T_tot);
    
    n = 500; % wasteful to plot all the points
    sel = round(linspace(1,length(T)-1,n));
    T = T(sel);
    th_ctrl = th_ctrl(sel,:);
    F = F(sel,:);

    %% Plot trajectories
    disp('Done!')
    
    [th_ref,v,a] = lspb_get(T, lspb_cfg);
   
    % Plot profiles
    h = [h,plotLSPBProfiles(robot, lspb_cfg, T, th_ref, v, a)];
    h = [h,plotCtrlProfiles(T, th_ctrl, F, th_ref)];
    
    % In Cspace
    h = [h,plot(cspace, th_ref(:,1),th_ref(:,2),'g')];
    h = [h,plot(cspace, th_ctrl(:,1),th_ctrl(:,2),'k')];

    % In Wspace
    xy_ref = forwardKin(robot, th_ref);
    h = [h,plot(wspace,xy_ref(:,1),xy_ref(:,2),':g')];
    xy_ctrl = forwardKin(robot, th_ctrl);
    h = [h,plot(wspace,xy_ctrl(:,1),xy_ctrl(:,2),':k')];
    drawnow
    figure(get(cspace,'parent')) % bring to front
    
    % Plot animations
    disp('Animation begins.')
    r = robotics.Rate(10);
    cnt = 0;
    for i = round(linspace(2,length(T)-1,100))
        hold on; 
        cnt = cnt + 1;
        h_robot = [];
        h_ref = drawRobot(wspace, robot, th_ref(i,:), 'k');
        h_ctrl = drawRobot(wspace, robot, th_ctrl(i,:), 'c');
        drawnow
        waitfor(r);
        if display_intermediate_anim && (mod(cnt,10) == 0) && (cnt ~= 100)
            h = [h, h_ref, h_ctrl];
        else
            delete(h_ref)
            delete(h_ctrl)
        end
    end
    
    axes(cspace);
    disp('> Please press a key to continue.')
    while ~waitforbuttonpress, end
    if dialogGUI(), continue; else, break; end
end

disp('Exited.')