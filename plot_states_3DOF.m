function [] = plot_states_3DOF(t,x) 
%%plot_states_3DOF(t,x) function to plot the states of 3DOF underwater
%%vehicle. 
%
% Created: Abhi Shah
%
% Inputs: t  time series data
%         x  time series state data

%%
subplot(221)
plot(t, x(:,1:2))
title 'Postion in World Frame'
legend('$x$','$y$');
xlabel 'time (s)';
ylabel 'postion (m)';
grid minor;

subplot(222)
plot(t, x(:,3)*180/pi)
title 'Orientation in World Frame'
xlabel 'time (s)';
ylabel 'angle (degrees)';
grid minor;

subplot(223)
plot(t, x(:,4:5))
title 'Linear Velocity in Body L-Frame'
legend('$v_x$','$v_y$');
xlabel 'time (s)';
ylabel 'velocity (m/s)';
grid minor;

subplot(224)
plot(t, x(:,6)*180/pi)
title 'Angular Velocity in Body L-Frame'
xlabel 'time (s)';
ylabel 'angular velocity (deg/s)';
grid minor;
end