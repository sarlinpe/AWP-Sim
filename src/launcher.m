% Title:        ME5402 Project 1-2: Trajectory Planning
% File:         launcher.m
% Date:         2017-04-20
% Author:       Nicolai Domingo Nielsen (A0164015R)
%               Paul-Edouard Sarlin (A0153124U)
% Description:  Launch and configuration file for the simulation 
%               and the interface.

clear; close all;
addpath('./path_finder', './trajectory_generator', './controller');

%% User interface
display_reachable_ws = true;        % takes significant time to compute...
display_intermediate_anim = true;  % intermediate configurations of the arm
display_waypoints = true;           % A* and pruned waypoints
show_prm = false;                   % PRM nodes and edges

%% Robot configuration

% Kinematics
robot.l1 = 6.5;         % length of link 1 [m]
robot.l2 = 7.5;         % length of link 2 [m]
th1_lim = [0,pi];       % limits of joint 1 [rad]
th2_lim = [-pi, pi];    % limits of joint 2 [rad]
th_init = [pi/3, -pi/4];% initial joint positions    

% Dynamics
robot.me = 150;         % end effector mass [kg]
rho_steel = 7850;       % density of steel [kg/m^3]
acr1 = 0.0625;          % cross section area of stair 1 [m^2]
acr2 = 0.0400;          % cross section area of stair 2 [m^2]

v_max = [0.1,0.1];      % max velocity of each joint [rad/s]
a_max = [0.005,0.005];  % max acceleration of each joint [rad/s^2]

%% Environment

% Add new circular obstacles: createCircle([x,y], radius) [m]
obstacles = [createCircle([5,10],0.5), ...
             createCircle([-10,5],1.2), ...
             createCircle([-2,13],0.8), ...
             createCircle([-5.2,8],0.8), ...
             createCircle([9,4.5],1)];

%% Algorithms

% Path finding
cs_resolution = 200;    % resolution of the c-space grid [cells/rad]
inflate_rad = 0.05;     % radius of inflation of the c-space [rad]
nb_prm_nodes = 2000;    % number of nodes of the PRM graph
prm_dist = 0.5;         % PRM connectivity radius [rad]

% PID controller gains
PID.Kp1 = 17;
PID.Kd1 = 9;
PID.Ki1 = 11;
PID.Kp2 = 18;
PID.Kd2 = 13;
PID.Ki2 = 12;

%% Launch
main;