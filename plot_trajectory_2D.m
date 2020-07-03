function [] = plot_trajectory_2D(t,x,u,s,S)
%%plot_trajectory_2D(t,x,u,s,S) function that plots the 2D trjectory of an
%%underwater vehicle.
%
% Created: Abhi Shah
%
% Inputs: t  time vector time series
%         x  state vector time series
%         u  control input time series
%         s  scale foctor for vehicle graphic
%         S  struct containing other vehicle parameters
%
% Output: Creates avi file with simulation animation
%
% Comments: Make sure text interpreter is set to latex
%           You can disable the legends to make the simulation run much
%           faster.
%
% TODO: any changes based on LLW feedback. Moving axes?
%%
%%%%%%%%%%%%%%%%%%%%% Configure Figure %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v = VideoWriter('~/3dof_vehicle_sim/Videos/testing.avi');  %location to save video
%v.FrameRate = 20;   %set frame rate for video writer
open(v);
fig = figure();
fig.WindowState = 'maximized';
f_int = 5; % choose plotting interval integer{1 length(t)}
n_thrusters = length(u(1,:));   % no. of thrusters

% Subplot locations
subplot_trajectory = [1, 6];
subplot_states = [3, 4, 7, 8];
subplot_control_forces = [9, 10];
if(S.getNet==1 || n_thrusters==3)  %if this additional plot is required
    subplot_control_torques = [11, 12];
end

%%%%%%%%%%% Initial Trajectory Subplot %%%%%%%%%%%
subplot(3,4,subplot_trajectory)
hold on;
tank_axes = [-3.5 4.5 -3.5 3.5];
axis(tank_axes);   % choose based on trajectory
p_tank = plot_tank();
p_desired = plot(S.xd(2),S.xd(1), '*r', 'LineWidth', 1);
p_trajectory = plot(x(1,2), x(1,1), '.b', 'LineWidth', 1);
p_initial = plot(x(1,2), x(1,1), '*b', 'LineWidth', 1);
title 'World Frame Vehicle Trajectory';
xlabel 'y-axis, position (m)';
ylabel 'x-axis, position (m)';
axis equal; grid minor;
P = plot_graphic_JHUROV(x(1,:),s,S.l);

% Annotate subplot
txt = ['Time = ',num2str(t(1)), 's'];
h = text((tank_axes(1)+.2),(tank_axes(4)-.2),txt);
h.FontSize = 15;
txt1 = ['Initial Position: (',num2str(x(1,1)), ',',num2str(x(1,2)),')'];
txt2 = ['Set Point:      (',num2str(S.xd(1)), ',',num2str(S.xd(2)),')'];

legend([p_initial, p_desired, P{1}, p_tank, p_trajectory],txt1, txt2,...
    'Vehicle', 'Tank','Trajectory','AutoUpdate','off');   %selective legend

%%%%%%%%%%%% Initial States Subplots %%%%%%%%%%%%%
subplot(3,4,subplot_states(1))
hold on;
plot(t(1), x(1,1), '.r');
plot(t(1), x(1,2), '.g');
title 'Postion in World Frame';
legend('$x$','$y$', 'AutoUpdate', 'off');
xlabel 'time (s)';
ylabel 'postion (m)';
grid minor;

subplot(3,4,subplot_states(2))
hold on;
plot(t(1), x(1,3)*180/pi, '.b');
title 'Orientation in World Frame';
xlabel 'time (s)';
ylabel '$\theta$ (degrees)';
grid minor;

subplot(3,4,subplot_states(3))
hold on;
plot(t(1), x(1,4), '.r');
plot(t(1), x(1,5), '.g');
title 'Linear Velocity in Body L-Frame';
legend('$v_x$','$v_y$', 'AutoUpdate', 'off');
xlabel 'time (s)';
ylabel 'velocity (m/s)';
grid minor;

subplot(3,4,subplot_states(4))
hold on;
plot(t(1), x(1,6)*180/pi, '.b');
title 'Angular Velocity in Body L-Frame';
xlabel 'time (s)';
ylabel '$\omega$ (deg/s)';
grid minor;

%%%%%%%%%% Initial Controls Subplot(s) %%%%%%%%%%%%
if(S.getNet==1)   %plot net thrusts
    subplot(3,4,subplot_control_forces);
    hold on;
    plot(t(1), u(1,1), '.r');
    plot(t(1), u(1,2), '.g');
    title 'Net Forces in Body L-Frame';
    legend('$f_x$','$f_y$','AutoUpdate','off');
    xlabel 'time (s)';
    ylabel 'force (N)';
    grid minor;
    
    subplot(3,4,subplot_control_torques);
    hold on;
    plot(t(1), u(1,3), '.b');
    title 'Net Torgue in Body L-Frame';
    xlabel 'time (s)';
    ylabel 'torque (Nm)';
    grid minor;
else  %plot individuals thrusts
    subplot(3,4,subplot_control_forces);
    hold on;
    plot(t(1),u(1,1), '.r');
    plot(t(1),u(1,2), '.g');
    plot(t, S.max_thrust*ones(length(t)), '--b');
    plot(t, -S.max_thrust*ones(length(t)), '--b');
    control_forces_axes = [0, t(end), (-S.max_thrust-10), (S.max_thrust+10)];
    axis(control_forces_axes);   %set axes for thrust forces 
    title 'Individual thruster forces'
    legend('$u_1$','$u_2$','$u_{max}$','AutoUpdate','off');
    xlabel 'time (s)';
    ylabel 'force (N)';
    grid minor;
    hu{1} = text(0.85*t(end),-100,['$u_1$ = ', num2str(u(1,1)),'N']);
    hu{2} = text(0.85*t(end),-125,['$u_2$ = ', num2str(u(1,2)),'N']);
    
    if(n_thrusters==3)  % fully actuated case - pure torque thruster
        subplot(3,4,subplot_control_torques)
        hold on;
        plot(t(1), u(1,3),'.b');
        title 'Individual thruster torque'
        xlabel 'time (s)';
        ylabel 'torque (Nm)';
        grid minor;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%% Loop over all Subplots %%%%%%%%%%%%%%%%%%%%%%%%%
for j = 1:f_int:length(x)
    %%%%%% Vehicle Trajectory %%%%%%%
    subplot(3,4,subplot_trajectory)
    for i = 1:length(P)
        delete(P{i});   %delete previous vehicle graphic
    end
    plot(x(j,2), x(j,1), '.b', 'LineWidth', .1);
    P = plot_graphic_JHUROV(x(j,:),s,S.l);
    axis(tank_axes); 
    txt = ['Time = ',num2str(t(j)), 's'];
    delete(h);   %delete previous annotation
    h = text((tank_axes(1)+.2),(tank_axes(4)-.2),txt); %display elapsed time
    h.FontSize = 14;
    
    %%%%%%%%%%%%% States %%%%%%%%%%%%%%
    subplot(3,4,subplot_states(1))
    plot(t(j), x(j,1), '.r');
    plot(t(j), x(j,2), '.g');
    
    subplot(3,4,subplot_states(2))
    plot(t(j), x(j,3)*180/pi, '.b');
    
    subplot(3,4,subplot_states(3))
    plot(t(j), x(j,4), '.r');
    plot(t(j), x(j,5), '.g');
    
    subplot(3,4,subplot_states(4))
    plot(t(j), x(j,6)*180/pi, '.b');
    
    %%%%%%%%%%%%% Controls %%%%%%%%%%%%%
    subplot(3,4,subplot_control_forces);
    plot(t(j), u(j,1), '.r');
    plot(t(j), u(j,2), '.g');
    delete(hu{1}); %delete previous annotation
    delete(hu{2}); %delete previous annotation
    hu{1} = text(0.85*t(end),-100,['$u_1$ = ', num2str(u(j,1)),'N']);
    hu{2} = text(0.85*t(end),-125,['$u_2$ = ', num2str(u(j,2)),'N']);
    if(n_thrusters==3)
        subplot(3,4,subplot_control_torques);
        plot(t(j), u(j,3), '.b');
    end
    
    % Capture frame at the end of all plots
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);
hold off;
end