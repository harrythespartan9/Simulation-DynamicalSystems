% This script runs an animation for a simplified single plane quadcopter.
% Here, the model simulates two sets of thrusters each on different ends of
% the base.
% Also please NOTE that the simulation starts from a point of equilibrium
% and simulates the LINEARIZED version of the system (based on Taylor's
% series expansion). You can modify the code to simulate the non-linear
% system.

% Written By: Hari Krishna Hari Prasad
% Affili    : Autonomous Insect Robotics Laboratory, ME, UW.

%% System Parameters and constants:
clear all; close all; clc;
%%%%%% CHANGE PARAMETERS HERE %%%%%%
m               = 1;            % Mass of the drone in Kg.
I               = 0.01;         % Roll axis MOI about COM in N*m/(rad/s^2).
b               = 0.1;          % Air damping - N/(m/s).

% Physics constants:
g           = 9.81;             % Acceleration due to gravity (kg*m/s^2).

% Simulation parameters:
T           = 0:0.01:2;           % Starting and ending time.

%% Defining Left and Right thrusts as functions of Time:
%%%%%%%% CHANGE ROLL LEFT AND RIGHT HERE %%%%%%%%%
k = 0;          % Simulation to roll left by default -> k = 1.
                % Simulation to roll right           -> k = 0.
                
%% SIMULATION:
% Here, we simulate the system and animate the result.
% Initial condition - the drone starts from rest:
X0          = zeros(6,1);

% Solve the ODE using ode45:
[t,X] = ode45(@(t,X) DerivSys(t,X,m,b,I,g,k), T, X0);

% Plot the results:
figure()
title('Trajectory taken by drone COM');
grid on;
plot(X(:,1),X(:,2),'r','LineWidth',1.2);
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');

% Get max&min X and Y positions:
x_max = max(X(:,1)); x_min = min(X(:,1));
y_max = max(X(:,2)); y_min = min(X(:,2));

%% Animating the results:
figure()
grid on;
for i = 1:length(t)
    if mod(i,2) == 0 || i == length(t)
        drawSys(X(i,:),x_max,y_max,x_min,y_min);
        if i ~= length(t)
            clf;
        end
    end
end

%% Function to compute the state-derivative data:
% We have defined our states as, X = [h; v; theta; h_dot; v_dot; theta_dot];
% We have defined our input as [T_r; T_l; g]
function dXdt = DerivSys(t,X,m,b,I,g,k)
% Defining our Input vector:
[T_r, T_l]      = thrustComp(k,m,g,t);
U               = [T_r; T_l; g];
% Let us define a variable which will mate our subsequent representation
% easier:
K               = (T_r + T_l)/m;
% The system matrix is given by:
A               =       [zeros(3,3),eye(3,3);
                        zeros(2,2),[K*cos(X(3)), -(b/m);
                                    -K*sin(X(3)), 0], [0, 0;
                                                      -(b/m), 0 ];
                        zeros(1,6)];
% The input matrix is given by:
B               =       [zeros(3,3);
                        [sin(X(3)), sin(X(3)), 0;
                         cos(X(3)), cos(X(3)), -1]
                         1/I, -(1/I), 0];
% Now, the state-derivative X_dot is given by:
dXdt            =       A*X + B*U;
end

%% Function to compute the thrust data:
% Here we compute the thrusts needed to roll the drone left or right.
function [TR, TL] = thrustComp(k,m,g,t)
if k ~= 1 % (Roll left)
    TR          = 5*m*g + 1.048/(1 + exp(-t));
    TL          = 5*m*g + 1/(1 + exp(-t));
else % (Roll right)
    TR          = 5*m*g + 1/(1 + exp(-t));
    TL          = 5*m*g + 1.048/(1 + exp(-t));
end
end

%% Animation drawing function.
% This function draws the system at a specific time-step.
function drawSys(X,x_max,y_max,x_min,y_min)
% Drawing the base:
line([(X(1) - 2*cos(-X(3))),(X(1) + 2*cos(-X(3)))], [(X(2) - 2*sin(-X(3))),...
    (X(2) + 2*sin(-X(3)))], 'Color','k','LineWidth',1.3);
hold on;
plot([(x_min-10),(x_max+10)],[0,0],'Color','b');
xlim([(x_min-10) (x_max+10)]);
ylim([(y_min-10) (y_max+10)]);
% Drawing the thrusters:
rectangle('Position',[(X(1) - 2*cos(-X(3)) -0.5) (X(2) - 2*sin(-X(3)) -0.5) 1 1],'FaceColor','r','Curvature',[1 1]);
rectangle('Position',[(X(1) + 2*cos(-X(3)) -0.5) (X(2) + 2*sin(-X(3)) -0.5) 1 1],'FaceColor','b','Curvature',[1 1]);
daspect([1,1,1])
set(gca,'XTickLabel',[],'YTickLabel',[],'xtick',[]);
drawnow;
hold off;
end 
