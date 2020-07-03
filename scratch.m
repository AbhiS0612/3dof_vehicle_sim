%% For arbititrary test scripts
close all;
syms m1 m2 J l p1 p2 theta v1 v2 v3 d mi1 mi2 mi3 a real
x = [p1 p2 theta v1 v2 v3]';
M = [m1 0 0;
    0 m2 -m2*l;
    0 -m2*l J];

ml = [0;-m2*l];
cl = [-m2*v2 + m2*l*v3; m1*v1];
C = [[0 0;0 0] cl;
      cl'      0];
D = diag([d*v1 d*v2 d*v3]);

%Mi = inv(M);
Mi = [mi1 0 0;
      0 mi2 a;
      0 a  mi3];
      
v = [v1 v2 v3]';
Rb = [-sin(theta)  cos(theta) 0;
      -cos(theta) -sin(theta) 0;
           0           0      1];
R = [cos(theta) -sin(theta) 0;
     sin(theta) cos(theta)  0;
        0           0       1];
       
% a =  Rb*v; 
% b =  -Mi*(C + D);
% z = zeros(3,1);
% g0 = [R*v; b*v];
%       
% g1 = [z;1/m;0;0]
% g2 = [z;0;0;1/J]
% 
% dg0_dx = jacobian(g0,x)
% 
% g3 = dg0_dx*g1
% g4 = dg0_dx*g2
% 
% dg3_dx = jacobian(g3,x)
% dg4_dx = jacobian(g4,x)
% g5 = dg0_dx*g3 - dg3_dx*g0  % [g0, [g0,g1]]
% g6 = dg0_dx*g4 - dg4_dx*g0  % [g0, [g0,g2]]

%rank([g1,g2,g3,g4,g5,g6])
v_dot = -Mi*(C+D)*v;
v_d = v_dot(1:2,:);
simplify(v_d)
V = [v1 v2]*v_d;
simplify(V)