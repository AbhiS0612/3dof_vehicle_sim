function [f, u] = control_3DOF_UA(t,x,S)
%control_3DOF_UA  Control forces for motion in 3 DOF in the under-actuated
%  case. This function computes the forces and torques on the l-frame of the
%  vehicle.
%  Inputs: t  time 
%          x  state vector of dim (6,1) = [px, py, theta, vx, vy, w]
%             (px,py) and theta are positon and orientation in world frame
%             respectively and (vx,vy) and w are linear an angular
%             velocities in body-frame frame
%          S  matlab struct containing lumped parameters. S.M is 3x3
%             symmetric positive definte mass matrix, S.D is a cell array
%             containing 3 quadratic drag matrices of dim 3x3. The drag
%             matrices are labelled S.D{i}, for i = 1,2,3 D.3. S.b is bias
%             term of dim 3x1, defaults to zero if no bias. S.l is the 2 
%             dimensional vector from the body frame to the l-frame 
%             defaults to zero, i.e COM frame if not present.
%
%  Outputs: Net control effort f, dim 3x1
%           Truster forces u, dim 2x1
%
%  Author: Abhi Shah
%  Created: 06-01-20
% 
%  Comments: 

%  TODO: - add check to ensure thrust is achievable 
%        - experient with different controllers

%% Choose Set-point
% Desired State in world frame
pos_d = S.xd(1:2,:);  %in meters
theta = x(3,1);
Rt = [cos(theta) sin(theta)
    -sin(theta) cos(theta)];
vel_d = S.xd(3:4,:);  %in m/s

%% Error Coordinates
err_pos = Rt*(x(1:2,1) - pos_d);
err_vel = x(4:5,1) - vel_d;
%% Choose Control law
Kp = 30*eye(2);
Kd = 30*eye(2);
fu = -Kp*err_pos - Kd*err_vel;
T = [1 0;
     0 0;
     0 1];
f = T*fu;

u = 0.5*[(fu(1) - fu(2)/S.cu);
         (fu(1) + fu(2)/S.cu)];
     
% 
end
