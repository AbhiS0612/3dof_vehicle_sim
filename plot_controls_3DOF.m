function [] = plot_controls_3DOF(t,u,net,t_max)
%%plot_controls_3DOF(t,u,net,t_max) plot the net forces and torques acting
%%along the 3 degrees of freedom. Can be used in fully actuated or
%%underactuated case.
%
% Created: Abhi Shah
%
% Inputs: t      time series data
%         u      time series of contorls. Can be for fully actuated or
%                underactuated, plotting, and to plot net contorls or
%                individual thruster forces
%         net    flag variable, 1 to plot net forces, 0 to plot individual
%                thrust forces
%         t_max  max permitted thrust force
%
%%
if(net==1)  %plot net forces - same for FA and UA cases.
    subplot(211)
    plot(t, u(:,1:2))
    title 'Net Forces in Body L-Frame'
    legend('$f_x$','$f_y$');
    xlabel 'time (s)';
    ylabel 'force (N)';
    grid minor;
    subplot(212)
    plot(t, u(:,3))
    title 'Control Torgue in Body L-Frame'
    xlabel 'time (s)';
    ylabel 'torque (Nm)';
    grid minor;
    
else  %plot individuals thrust forces
    n_thrusters = length(u(1,:));
    
    if(n_thrusters == 2)   %underactuated case
        plot(t, u(:,1:2))
        hold on;
        plot(t, t_max*ones(length(t)), '--r');
        plot(t, -1*t_max*ones(length(t)), '--r');
        hold off;
        title 'Individual thruster forces'
        legend('$u_1$','$u_2$', '$u_{max}$');
        xlabel 'time (s)';
        ylabel 'force (N)';
        grid minor;
        
    else  %fully actuated case
        subplot(211)
        plot(t, u(:,1:2))
        hold on;
        plot(t, t_max*ones(length(t)), '--r');
        plot(t, -1*t_max*ones(length(t)), '--r');
        hold off;
        title 'Individual thruster forces'
        legend('$u_1$','$u_2$', '$u_{max}$');
        xlabel 'time (s)';
        ylabel 'force (N)';
        grid minor;
        subplot(212)
        plot(t, u(:,3))
        title 'Individual thruster torque'
        xlabel 'time (s)';
        ylabel 'torque (Nm)';
        grid minor;
    end
end
end