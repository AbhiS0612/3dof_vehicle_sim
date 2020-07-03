function [xdot, f] = DYN_3DOF(t, x, S)
%DYN_3DOF: Fuction containing the kinematics and dynamics of a 3 DOF
%(motion in a horizontal plane) autonomous underwater vehicle. Call this
%using ode45. 
% 
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
%  Outputs: xdot  the derivative of state x
%
%  Author: Abhi Shah
%  Created:   05-29-20
%  Modified:  06-01-20 (Abhi Shah)
% 
%  Comments: 

%  TODO: - More testing
%        - Use realistic model parameters
%        - Work on control law
%
%% 
% px = x(1,1); py = x(2,1), 
 theta = x(3,1);
 vx = x(4,1); vy = x(5,1); w = x(6,1);

 % Kinematics in COM frame and in l frame are identical
 R = [cos(theta)  -sin(theta)  0
      sin(theta)   cos(theta)  0
          0           0        1];
 
 xdot(1:3,1) = R*x(4:6,1);
 
 % Dynamics
 % Compute coriolis and mass matrix 
  M_t = S.M(1:2,1:2);
  M_l = M_t*J2(-1)*S.l;
  
  C = [    J2(w)*M_t           J2(w)*M_l;
       [vx  vy]*M_t*J2(1)  [vx vy]*J2(-1)*M_l];
  
  % additional intertia in the l frame  
  i_l = [S.l(1) -S.l(2)]*M_t*[S.l(1);-S.l(2)];
   
  %Mass matrix in l-frame
  M_T = [M_t     M_l;
         M_l'    S.M(3,3)+i_l];
     
 % Compute drag matrices
  DQ = S.D{1}*abs(vx) + S.D{2}*abs(vy) + S.D{3}*abs(w);
  
  % Get control forces and torques on l-frame
  [f_net, f_thrust] = control_3DOF_UA(t,x,S);  
  if(S.getNet==1)
    f = f_net;
  else
    f = f_thrust;
  end
  
  xdot(4:6,1) = M_T\(f_net) - M_T\( (C + DQ)*x(4:6,1) - S.b );
end