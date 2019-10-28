% This script runs an animation for a car suspension (mass-spring- damper)
% system based on a highly simplified dynamical model reacting to impulsive 
% distrubances.

% Written By: Hari Krishna Hari Prasad
% Affili    : Autonomous Insect Robotics Laboratory, ME, UW.

%% System Parameters and constants:
clear all; close all; clc;
%%%% CHANGE PARAMETERS HERE %%%%
m           = 10;          % Mass of the car in Kg.
k           = 1;          % Spring constant in Kg/m.
b           = 2;%5;%2;%0.1;           % Damping ration in Kg/(m/s).

% Physics constants:
g           = 9.81;         % Acceleration due to gravity (kg*m/s^2).

% Simulation parameters:
tfinal      = 100;         
T           = 0:0.5:tfinal;   % Starting and ending time.

%% State-space representation for the system:
% We have defined our states as, X = [y; y_dot];
% Hence our state-derivative will be for the form, X_dot = [y_dot; y_ddot];
% Here, dot represents a single time dervative, and ddot represents two
% time derivatives - velocity and acceleration.

A           = [0 1; -(k/m) -(b/m)];         % System matrix.      
B           = [0; (1/m)];                   % Input matrix.
C           = [1 0; 0 1];                   % Output/Sensor matrix.
% Basically, we are only measuring our position as a function of time.
D           = 0;                            % Feedfwd matrix (usually 0).

% Let us now initialize the State-Space variable:
sys         = ss(A,B,C,D);                  % Our system as a St-Sp var.

%% SIMULATION:
% Here, we simulate our system where a huge speedbreaker hits our car at
% the beginning (Impulsive force), let us look at the response.
Y = impulse(sys,T);

% For this we will add the system steady state initially to show the
% difference:
T     = -50:0.5:tfinal;                % Updated.
y = zeros(length(T),1);
y_dot = zeros(length(T),1);

for j = 1:length(T)
    if j < 101
        y(j,1) = 0;
        y_dot(j,1) = 0;
    else
        y(j,1) = Y(j-100,1);
        y_dot(j,1) = Y(j-100,2);
    end
end

% % Plotting results:
% figure()
% plot(0.1*T,y, 'r', 'LineWidth',1.2);
% hold on; grid on;
% %plot(T,y_dot, 'b', 'LineWidth',1.2);
% xlabel('Time (s)');
% ylabel('Y pos (m)');
% %ylabel('Y pos/vel (m or m/s)');
% title('Y data vs Time');
% %legend('Y-position', 'Y-Velocity');

%% ANIMATION:
% Here we visualize our data through a simple animation.
% Setup figure:
figure()
grid on;
for i = 1:length(T)
    if mod(i,2) == 0 || i == length(T)
        drawSys_SpringMassDamper(y(i,1));
        if i ~= length(T)
            clf;
        end
    end
end
