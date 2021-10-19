% MCS Project SS2021
% Inverted Compund Pendulum
% Prof. Antonello Monti
% Project Contact: Sriram Gurumurthy [SGurumurthy@eonerc.rwth-aachen.de]
%% Setup
clear;
clc;
Ts = 5e-5;  % Simulation time step
T_sample = 0.01; % % Control sample time
%% Inverted Compound Pendulum Plant Constants 

% Parameters that don't change 

Plant1.m_r = 1;                                 % Mass of the rod [Kg]
Plant1.m_d = 3;                                 % Mass of the disc [Kg]
Plant1.l   = 0.4;                               % Length of the rod [m]
Plant1.R_d = 0.02;                              % Radius of circular disc [m]
Plant1.g   = 9.81;                              % acceleration due to gravity [m/s^2]
Plant1.T_md = 0.02;                             % Denominator Time constant of closed loop torque control of motor [s]
Plant1.T_mn = 0.04;                             % Numerator Time constant of closed loop torque control of motor [s]


% Parameters that can be modified for simulations
Plant1.Motor_select = 1;                        % 0 - Motor model is not seledcted, 1 - Motor model is selected
Plant1.Noise_select = 1;                        % 0 - Noise model is not seledcted, 1 - Noise model is selected
Plant1.c   = 0.1;                               % Coefficient of air friction [Kg m^2 / s], For testing LQG in Task 3, c can be chosen between 0.05 to 0.2 [Kg m^2 / s].
%% System Constants

c   = 0.1;                              % Coefficient of air friction [Kg m^2 / s]
Inertia_model = 1;

switch Inertia_model
    
    case 1 % Considering the correct model (rod with a rotating disc)
        
        J = (Plant1.m_r/3 + Plant1.m_d)*Plant1.l^2 + Plant1.m_d*Plant1.R_d^2/2 ;    % Inertia of the compund pendulum [Kg m^2]
        
    case 2 % Considering that the total mass as a point mass connected to the end of a massless rod
        
        M_total = Plant1.m_r + Plant1.m_d ;
        J = M_total * Plant1.l^2  ;                                                 % Inertia of the pendulum with massless rod and a point mass [Kg m^2]      
        
end

T_dist = -75;                              % Disturbance Torque [Nm]  Upto +-75 Nm
T_sat = 20;                                % Saturation torque applied for the motor (upper limit +T_sat, lower limit -T_sat) [Nm]
%% Initial conditions
Angle_start_up            = pi-0.001;                    % Inverter position, x1 is pi radians
Angular_velocity_start_up = 0;                           % Angular velocity is zero
x_init = [Angle_start_up; Angular_velocity_start_up] ;   % Initial condition vector   
   
%% Control Task 1 
% Control Task 1.1 
A1_21 = ((Plant1.m_r/2) + Plant1.m_d)*Plant1.g*Plant1.l/J;
A1_22 = -c/J;

A1 = [0 1; A1_21 A1_22];
B1 = [0;1/J];
C1 = [1 0];
D1 = 0;
    
sys = ss(A1,B1,C1,D1,'StateName',{'Theta' 'Angular Velocity'},...
 'InputName','Tu','OutputName','Theta');

p1 = pole(sys);
z1 = zero(sys);
pzmap(sys);
%.....

% Control Task 1.2 
% Change the Inertia_model parameter to 2 for this task chekc the pole-zero
% plot of the system for the assumption.
%.....

% Control Task 1.3 
sys_ss1 = c2d(sys,T_sample);

poles1 = eig(sys_ss1.A);

if Plant1.Motor_select == 0
    co1 = ctrb(sys_ss1);
    controllability1 = rank(co1);

    if controllability1 == length(sys_ss1.A)
       Q = [2000 0;0 50];
       R  = 1;
      [KK,S,e] = dlqr(sys_ss1.A,sys_ss1.B,Q,R);
    end
    KK = [KK 0];
end
%.....


%% Control Task 2 

% Task 2.1
num = [-Plant1.T_mn 1];
den = [Plant1.T_md 1];

[A2,B2,C2,D2] = tf2ss(num, den);

tf_mt_cl = ss(A2,B2,C2,D2, 'StateName',{'Motor'},'InputName','Tc','OutputName','Tu');

sysac = series(tf_mt_cl,sys);

sys_ss2 = c2d(sysac,T_sample);

poles2 = eig(sys_ss2.A);

%.....

% Task 2.2

co2 = ctrb(sys_ss2);
controllability2 = rank(co2);

if Plant1.Motor_select == 1
    if controllability2 == length(sys_ss2.A)
       Q = [2000 0 0;0 50 0;0 0 50];
       R  = 1;
       [KK,S,e] = dlqr(sys_ss2.A,sys_ss2.B,Q,R);
    end
end
ob1 = obsv(sys_ss2);
observability1 = rank(ob1);
%.....

%% Control Task 3

% Task 3.1
Q=0 ; R = 1; N=0;

[kalmf,L,P] = kalman(sys_ss2,Q,R,N);
%.....

% Task 3.2
%To run the LQG filter, need to activate the plant model with noise and
%that can be done by setting the parameter Plant1.Noise_select to 1.

%The LQG Controller is implemeneted in the Simulink model with a LQR state
%feedback and a Kalman Filter to estimate the states.
%.....



