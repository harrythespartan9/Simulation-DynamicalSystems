% This script animates a Pogo stick which is a hybrid dynamical system -
% meaning there are two associated systems one when in the air and the
% other when contacting ground.

% Written By: Hari Krishna Hari Prasad
% Affili    : Autonomous Insect Robotics Laboratory, ME, UW.

%% System Parameters and constants:
clear all; close all; clc;
%%%%%% CHANGE PARAMETERS HERE %%%%%%
m               = 1;           % Mass of stick in Kg.
k               = 30;%30;            % Spring constant in N/m.
b               = 2;%2;            % Damping in the spring - N/(m/s)

% Physics constants:
g           = 9.81;             % Acceleration due to gravity (kg*m/s^2).

% Simulation parameters:
dt          = 0.001;            % Time step size (s).
T           = 0:dt:10;           % Time Vector (s).

%% State-space representation for the system:
% We have defined our states as, X = [y; y_dot];
% SYSTEM 2 - WHEN IN AIR.
A1           = [0 1; 0 0];
B1           = [0; -1];

% SYSTEM 2 - WHEN ON GROUND.
A2           = [0 1; -(k/m) -(b/m)];               
B2           = [0; -1];                                    

%% SIMULATION:
% Here we simulate the pogo stick.
% Initial condition - the Stick starts from rest:
Y0          = [10; 0];

% Initialize variables:
Y = zeros(2,length(T));
Y_ddot = zeros(1,length(T));

% Solve the ODE manually using a for loop:
for i = 1:(length(T)-1)
    if i == 1
        % Initialize the position and velocity based on ICs given:
        Y(1,i) = Y0(1,1);
        Y(2,i) = Y0(2,1);
        if Y(1,i) < 5
            Y_ddot(i) = (k/m)*(5-Y(1,i)) - (b/m)*Y(2,i) - g;  % Ground sys2.
            Y(2,i+1)    = Y(2,i) + Y_ddot(i)*dt; % Update velocity.
            Y(1,i+1)    = Y(1,i) + Y(2,i)*dt;    % Update position.
        else
            Y_ddot(i) = -g; % Freefall sys1.
            Y(2,i+1)    = Y(2,i) + Y_ddot(i)*dt; % Update velocity.
            Y(1,i+1)    = Y(1,i) + Y(2,i)*dt;    % Update position.
        end
    else
        if Y(1,i-1) < 5
            Y_ddot(i) = (k/m)*(5-Y(1,i)) - (b/m)*Y(2,i) - g;  % Ground sys2.
            Y(2,i+1)    = Y(2,i) + Y_ddot(i)*dt; % Update velocity.
            Y(1,i+1)    = Y(1,i) + Y(2,i)*dt;    % Update position.
        else
            Y_ddot(i) = -g; % Freefall sys1.
            Y(2,i+1)    = Y(2,i) + Y_ddot(i)*dt; % Update velocity.
            Y(1,i+1)    = Y(1,i) + Y(2,i)*dt;    % Update position.
        end
    end
end

% Get min Y position:
y_max = max(Y(:,2));

%% Animating the results:
figure()
grid on;
for i = 1:(length(T)-1)
    if mod(i,50) == 0 || i == (length(T)-1)
        drawSys(Y(1,i),y_max);
        if i ~= (length(T)-1)
            clf;
        end
    end
end

%% Animation drawing function.
% This function draws the system at a specific time-step.
function drawSys(y,y_max)
% Drawing the base:
if y > 5
    line([0,0],[y,y-5], 'Color','r','LineWidth',6);
else
    line([0,0],[y,0], 'Color','r','LineWidth',6);
end
hold on;
% Drawing the handle:
line([-1.5,1.5],[y,y], 'Color','b','LineWidth',10);
xlim([-10 10]);
ylim([-1 (y_max+5)]);
refline([0 0]);
% Drawing the bottom circle (bouncy part):
r = 0.5; % radius
if y > 5
    rectangle('Position',[-r (y-5) 2*r 2*r],'FaceColor','g','Curvature',[1 1]);
else
    rectangle('Position',[-r 0 2*r 2*r],'FaceColor','g','Curvature',[1 1]);
end
daspect([1,1,1])
set(gca,'XTickLabel',[],'YTickLabel',[],'xtick',[]);
drawnow;
hold off;
end