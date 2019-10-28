%% Animation drawing function.
% This function draws the system at a specific time-step.
function drawSys_SpringMassDamper(y_position)
% Offset the position of the car to (0,2.5):
yy = y_position + 0.8;
% Get the lower_left tip position of the car (black sqaure):
yp = yy - 0.25;
% Set the axis limits and other parameters needed:
xlim([-1.5 1.5]);
ylim([-0.5 1.5]);
% axis equal;
% Draw ground level:
refline([0 0]);
hold on; % Overlap subsequent plots.
% Drawing the car:
rectangle('Position',[-0.25 yp 0.5 0.5],'FaceColor','r');
% Displaying the shock-absorber:
rectangle('Position',[-0.075 0.15 0.15 yp],'FaceColor','g');
% We will also draw a small circle displaying the wheel which helps seeing
% the change in the y-position of the Car.
rectangle('Position',[-0.15 0 0.3 0.3],'FaceColor','b','Curvature',[1 1]);
drawnow;
hold off; % Stop overlapping subsequent plots.
end 