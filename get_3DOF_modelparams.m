function S = get_3DOF_modelparams()
%%get_3DOF_modelparams is used to save all the model parameters for simulation
%  of the dynamical model, namely lumped mass matrix, the 3 drag matrices,
%  the bias vector, if any, and the 2-dimensional vector from the COM to
%  the origin of the l-frame.
%  Inputs: none
%
%  Outputs: struct S
%
%  Author: Abhi Shah
%  Created: 06-01-20
% 
%  Comments: Could have different such files with model parameters of the
%  different vehicles saved, JHU-RIV, Iver etc
%            This file contains JHU-ROV params taken from Martin's 2012 paper,
%           Preliminary Experiments in Underactuated Nonlinear Model-Based 
%           Tracking Control of Underwater Vehicles with Three Degree-of-Freedom
%           Fully-Coupled Dynamical Plant Models: Theory and Experimental
%           Evaluation. Values rounded for clarity
%  TODO: -
%
%%
% Lumped parameter mass matrix in COM frame, dim 3x3
S.M = [520 -20  -5;
       -20 620 -40;
        -5 -40 110]; 
%S.M = diag([400 500 150]);
% 
% Quadratic drag matrices in l-frame, each of dim 3x3
% D1 = zeros(3);
% D1(1,1) = 1100;
% D2 = zeros(3);
% D2(2,2) = 2500;
% D3 = zeros(3);
% D3(3,3) = 280;

% 
% % Each drag matrix is diag - so coupled 
% D1 = diag([550 5 70]);
% D2 = diag([50 950 30]);
% D3 = diag([60 -30 200]);

% % Fully coupled drag
D1 = [570 1 65;
      -100 620 -160;
      75 -20 140];
D2 = [280 -50 120;
      50 950 30;
      50 -50 120];  
D3 = [500 -50 20;
      -80 970 -120;
      65 -30 215];

S.D = {D1, D2, D3};
% Constant bias vector in COM frame, dim 3x1 (optoinal)
S.b = zeros(3,1);
%S.b(3,1) = 1.5;

% Vector from COM to origin of l-frame, dim 2x1 (optional)
S.l = zeros(2,1);
S.l = [0.5;0];  % [1;0] does really well with UA_control

S.cu = 0.6;  % distance from body x axis to point of thrust application
end