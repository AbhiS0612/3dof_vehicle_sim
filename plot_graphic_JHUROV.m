function P = plot_graphic_JHUROV(x, s, l)
%plot_graphic_JHUROV plots a graphic of the vehicle at state x, of scale
%factor s, using model parameters specific by struct S 
%
% Created: Abhi Shah
%
% Inputs: x   current state
%         s   scale factor
%         l   vector from COM frame to l frame
%
% Output: P   struct containing various components plots
%
% Comments: To test this as stand-alone function you need to uncomment the
%           hold on and hold off actions below. To use with
%           plot_trajectory_2D.m, these two lines should be commented out.

%%
pos = [x(2) x(1)]; % l-frame position
R = [cos(x(3)) -sin(x(3));
     sin(x(3)) cos(x(3))];
com = pos - s*[l(2) l(1)]*R;
pc = plot(com(1), com(2), '+y', 'LineWidth', 2);
%hold on  %uncomment if testing this function independently
pl = plot(pos(1), pos(2), '+k', 'LineWidth', 1);
%vertices = [.4 .5; .4 -.6; -.4 -.6; -.4 .5; 0 .7]; % simplified model
vertices = [.4 .5; .4 -.4; .6 -.4; .6 -.5; .4 -.5; .4 -.6; -.4 -.6; -.4 -.5; -.6 -.5; -.6 -.4; -.4 -.4; -.4 .5; 0 .7];
vertices = vertices + com;
p = polyshape(vertices);
p = scale(p,s,com);
p = rotate(p, -x(3)*180/pi, com);
p_vehicle = plot(p);
axis equal;
grid on;
p_vehicle.FaceColor = [0.3290 0.2 0.1250];
p_vehicle.LineWidth = 2;
p_vehicle.FaceAlpha = 0.3;
P = {p_vehicle, pl, pc};
%hold off   %uncomment if testing this function independently
end