%% Main Simulation script for simulating 3DOF vehicle dynamics in an aribitary body fixed frame.
%
%  Author: Abhi Shah
%  Created: 06-01-20
%
%  Comments: 
%  - There doesn't really seem to be a great way to compute control
%    inputs, but current technique seems to work.
%  - Choose either static plots or the animation, but not both - Matlab
%    gets confused on messes things up.
%
%  TODO: make any changes based on LLW recommendations.

%% Configure Simulation
% Vehicle Parameters
S = get_3DOF_modelparams();
S.max_thrust = 150;
S.getNet = 0;       %1 to get net forces, 0 to get individual truster forces
vehicle_scale = 1;  % for drawing graphic only

% Initial Conditions
p0 = [-1;-1;0];     % initial postion (world frame)
v0 = [0;0;0];       % initial velocity (body l-frame)
x0 = [p0;v0];       % initial state
tspan = [0 60];     % simulation time span

% Desired Set Point
S.xd = [2;1;0;0];            %underactuated
% S.xd = [2;1;pi/2;0;0;0];   %fully actuated

% Configure Latex Interpreter - needs to be run once
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
%% Solve ODE
opt = odeset('RelTol',1e-10);  % set ode tolerance
[t,x] = ode45(@(t,x) DYN_3DOF(t,x,S),tspan,x0,opt);
[~,u] = cellfun(@(t,x) DYN_3DOF(t,x.',S), num2cell(t), num2cell(x,2), 'uni',0);  %get control inputs
u = cell2mat(u);
u = reshape(u,length(u)/length(t),[])';   %reshape control inputs

%% Plot Results - Static
% close all;
% figure
% plot_states_3DOF(t,x);
% 
% figure
% plot_controls_3DOF(t,u,S.getNet,S.max_thrust);

%% Plot Results - Animation
close all;
plot_trajectory_2D(t, x, u, vehicle_scale, S);
    