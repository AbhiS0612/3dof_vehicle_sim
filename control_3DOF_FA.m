function [f, u] = control_3DOF_FA(t,x,S)
%control_3DOF_FA  Control forces for motion in 3 DOF in the fully actuated
%  case. This function computes the forces and torques on the l-frame of the
%  vehicle.
%  Inputs: t  time 
%          x  state vector of dim (6,1) = [px, py, theta, vx, vy, w]
%             (px,py) and theta are positon and orientation in world frame
%             respectively and (vx,vy) and w are linear an angular
%             velocities in body-frame frame
%          S  matlab struct containing lumped parameters. S.M is 3x3
%          symmetric positive definte mass matrix, S.D is a cell array
%          containing 3 quadratic drag matrices of dim 3x3. The drag
%          matrices are labelled S.D{i}, for i = 1,2,3 D.3. S.b is bias
%          term of dim 3x1, defaults to zero if no bias. S.l is the 2 dimensional vector from the body
%          frame to the l-frame defaults to zero, i.e COM frame if not
%          present.
%
%  Outputs: Net control effort f, dim 3x1
%           Thruster forces u, dim 3x1
%
%  Author: Abhi Shah
%  Created: 06-01-20
% 
%  Comments: Assumes there is a force through COM along body x-axis u(1), a
%            force through COM along body y-axis u(2) and a torque u(3).
%            This setup is a little unusual perhaps, since the vehicle will
%            never have the ability to apply 2 forces and a torque
%            independently. But partly using this as sanity check. For non
%            zero S.l, notice that torque in COM frame and l-frame are not
%            the same.
%
%  TODO: - tune gains
%        
%% Choose Set-point
% Desired Postion of l-frame(m) in world frame
pos_d = [0;2];
% Desired Orientation of l-frame in world frame (degrees)
theta_d = 0;
theta_d = theta_d*pi/180;

% Postion error in l-frame
theta = x(3,1);
Rt_d = [cos(theta_d) sin(theta_d)
      -sin(theta_d) cos(theta_d)];
Rt = [cos(theta) sin(theta)
    -sin(theta) cos(theta)];
% Desired linear velocity in l-frame (m/s)
vel_d = [0;0];
% Desired angular velocity in l-frame (deg/s)
ang_vel_d = 0;
ang_vel_d = ang_vel_d*pi/180;

%% Error Coordinates
err_pos = (Rt*x(1:2,1) - Rt_d*pos_d);
err_theta = theta - theta_d;
err_vel = x(4:5,1) - vel_d;
err_ang_vel = x(6,1)- ang_vel_d;
%% Choose Control law
Kp = 25*eye(3);
Kd = 25*eye(3);
u = -Kp*[err_pos;err_theta] - Kd*[err_vel;err_ang_vel];

% Add additional torque due to forces in COM-frame
f = u + [0;0;-S.l(1)*u(2) + S.l(2)*u(1)]; 
end
