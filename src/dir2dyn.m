function ddq = dir2dyn(u);
% From http://www-lar.deis.unibo.it/people/cmelchiorri/Files_Robotica/FIR_05_Dynamics.pdf
% function ddq = dir2dyn(q,dq,tau,tauint);
% Dynamic model of a planar 2 dof manipulator
% Input:
% dq -->> joint velocity vector
% q -->> joint position vector
% tau -->> joint torques
% tauint -->> interaction forces
% Output:
% ddq -->> joint acceleration vector
global I1z I2z m1 m2 L1 L2 Lg1 Lg2 g D
q1 = u(1); q2 = u(2);
dq = u(3:4); dq1 = u(3); dq2 = u(4);
tau = u(5:6); tauint = u(7:8);
% Kinematic functions
s1 = sin(q1); s2 = sin(q2); s12 = sin(q1+q2);
c1 = cos(q1); c2 = cos(q2); c12 = cos(q1+q2);
%%%%% Elements of the Inertia Matrix M
M11 = I1z+I2z+Lg1^2*m1+m2*(L1^2+Lg2^2+2*L1*Lg2*c2);
M12 = I2z+m2*(Lg2^2+L1*Lg2*c2);
M22 = I2z+Lg2^2*m2;
M = [M11, M12; M12, M22];
%%%%% Coriolis and centrifugal elements
C11 = -(L1*dq2*s2*(Lg2*m2)); C12 = -(L1*dq12*s2*(Lg2*m2));
C21 = m2*L1*Lg2*s2*dq1; C22 = 0;
C = [C11 C12; C21 C22];
%%%%% Gravity :
g1 = m1*Lg1*c1+m2*(Lg2*c12 + L1*c1); g2 = m2*Lg2*c12;
G = g*[g1; g2];
%%%%% Computation of acceleration
ddq = inv(M) * (tau + tauint - C*dq - D*dq - G);
% Simulation of the dynamic model of a planar 2 dof manipulator
% Simulink scheme: sc_d2dof.m
% Definition and initialization of global variables
% Run the simulation RK45
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Robots? Coefficients %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear I1z I2z m1 m2 L1 L2 Lg1 Lg2 g D
global I1z I2z m1 m2 L1 L2 Lg1 Lg2 g D
I1z = 10; I2z = 10;
m1 = 50.0; m2 = 50.0;
L1 = 1; L2 = 1;
Lg1=L1/2; Lg2=L2/2;
g = 9.81;
D = 0.0 * eye(2); % friction
% D = diag([30,30]); % friction
% D = diag([80,80]); % friction
COPPIA = 0; % joint torques
DISTURBO = 0; % joint torques due to interaction with environment
%%%% Iniziatialization and run
TI = 0; TF = 10;
ERR = 1e-3;
TMIN = 0.002; TMAX = 10*TMIN;
OPTIONS = [ERR,TMIN,TMAX,0,0,2];
X0 = [0 0 0 0];
[ti,xi,yi]=rk45(?sc_d2dof?,TF,X0,OPTIONS);
D = diag[0, 0];