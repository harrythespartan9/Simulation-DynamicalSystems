% This script runs an animation for a simplified Elevator system. When the
% button to a specific floor is pressed, it is basically a step input and
% the reaction of the system to a step input is observed.

% Written By: Hari Krishna Hari Prasad
% Affili    : Autonomous Insect Robotics Laboratory, ME, UW.

%% System Parameters and constants:
clear all; close all; clc;
%%%%%% CHANGE PARAMETERS HERE %%%%%%
m               = 10;           % Mass of the elevator in Kg.
b               = 3;%200;%3;            % The Elevator shaft mechanism isn't
                                % friction/damping free - N/(m/s)

% Physics constants:
g           = 9.81;             % Acceleration due to gravity (kg*m/s^2).

% Simulation parameters:
T           = 0:0.05:100;        % Starting and ending time.

%% State-space representation:
% We have defined our states as, X = [y; y_dot];
% Hence our state-derivative will be for the form, X_dot = [y_dot; y_ddot];
% Here, dot represents a single time dervative, and ddot represents two
% time derivatives - velocity and acceleration.

% This A matrix is basically empty because there is not
% autonomous dynamics associated with the system.
A           = [0 1; 0 -(b/m)];              % System matrix.      
B           = [0; (1/m)];                   % Input matrix.
C           = [1 0; 0 1];                   % Output/Sensor matrix.
% Basically, we are only measuring our position as a function of time.
D           = 0;                            % Feedfwd matrix (usually 0).

% Let us now initialize the State-Space variable:
sys         = ss(A,B,C,D);                  % Our system as a St-Sp var.

% Initial condition for the system:
Y0          = [0; 0];                       % System starts from rest.

%% Uncontrolled constant force response (step response):
% Let's what happens when we give a constant force to the system (also
% called the step-response):

Y_uc = step(sys,T); % Here, uc stands for uncontrolled.

% % Plotting results:
% figure()
% plot(T,Y_uc(:,1), 'rx', 'LineWidth',1.2);
% hold on; grid on;
% plot(T,Y_uc(:,2), 'bo', 'LineWidth',1.2);
% xlabel('Time (s)');
% ylabel('Y pos/vel (m or m/s)');
% title('Y data (Uncontrolled) vs Time');
% legend('Y-position', 'Y-Velocity');
% % OR YOU CAN JUST SHOW HOW THE POSITION EXPONENTIALLY INCREASES.

%% PID CONTROL OF THE Y-VALUE:
% Here, we implement a basic PID controller to control the y-position (can
% be thought of as which floor we are in):

%%%%% CHANGE PID GAINS HERE!! %%%%%
% CRTICALLY DAMPED RESPONSE: %
Kp          = 10;               % Proportional Gain.
Ki          = 0.001;            % Integral Gain.
Kd          = 25;               % Derivative Gain.

% % OVERSHOOT AND OSCILLATING RESPONSE: %
% Kp          = 10;               % Proportional Gain.
% Ki          = 0.001;            % Integral Gain.
% Kd          = 5;                % Derivative Gain.

% Initialize Error_sum for integral correction:
global error_sum;
error_sum = 0;

% Get the set-point (based on the floor you want to go):
% 0 -> starting point (0m).
% 1 -> 5m.
% 2 -> 10m and so on.
yd = 10;                % 3rd floor.

% Let us now call the ODE solver to get the time-series results:
[T,Y] = ode45(@(T,Y) DerivSys(T,Y,A,B,Kp,Ki,Kd,yd), T, Y0);

% Let us look at the maximum and minimum y-position:
y_max = max(Y(:,1));
y_min = min(Y(:,2));

% Plotting the results:
% figure()
% ylim([(y_min-2) (y_max+2)]);
% grid on; hold on;
% plot(T,Y(:,1),'r','LineWidth',1.2);
% refline([0 yd]);
% plot(T,Y(:,2),'b','LineWidth',1.2);
% refline([0 0]);
% xlabel('Time (s)');
% ylabel('Vertical Height/Velocity (y in m || y_dot in m/s)');
% title('PID Control of Vertical Elevator Position');
% legend('Height (m)','Velocity (m/s)');

%% Animating the PID controlled system:
figure()
grid on;
set(gca,'XTickLabel',[]);
for i = 1:length(T)
    if mod(i,5) == 0 || i == length(T)
        drawSys(Y(i,1),y_max);
        if i ~= length(T)
            clf;
        end
    end
end

%% Animation drawing function.
% This function draws the system at a specific time-step.
function drawSys(y_position,y_max)
xlim([-2 2]);
ylim([-1 (y_max+2)]);
axis equal tight;
line([-1, -1], [-1, 20], 'Color', [0,1,0]);
hold on;
line([1, 1], [-1, 20], 'Color', [0,1,0]);
% Drawing the elevator:
rectangle('Position',[-0.5 y_position 1 1],'FaceColor','r');
set(gca,'XTickLabel',[],'YTickLabel',[]);
drawnow;
hold off;
end 

%% Function to compute the state-dervative data:
function dydt = DerivSys(~,y,A,B,Kp,Ki,Kd,yd)
% Call the global error sum variable:
global error_sum;
error_sum = error_sum + (yd - y(1));
% Computing the net force applied on the system by the controller:
F_net       = Kp*(yd - y(1)) + Ki*(error_sum) + Kd*(0 - y(2));
% Here our reference for velocity is zero. We want it to come to a rest.
% Finally computing the state derivative:
dydt        = A*[y(1); y(2)] + B*F_net;
end