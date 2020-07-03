function P = plot_graphic_Iver(x, s, S)
%plot_graphic_JHUROV plots a graphic of the vehicle at state x, of scale
%factor s, using model parameters specific by struct S 
%
% Created: Abhi Shah

% TODO - this function is not complete
%%
pos = [x(2) x(1)]; % l-frame position
R = [cos(x(3)) -sin(x(3));
     sin(x(3)) cos(x(3))];
com = pos - s*[S.l(2) S.l(1)]*R;
pp = plot(pos(1), pos(2), 'ok', 'LineWidth', 2);
%hold on
pc = plot(com(1), com(2), 'or', 'LineWidth', 2);
vertices = [.4 .5; .4 -.6; -.4 -.6; -.4 .5; 0 .8]; %
vertices = vertices + com;
p = polyshape(vertices);
p = scale(p,s,com);
p = rotate(p, -x(3)*180/pi, com);
pg = plot(p);
axis equal;
grid on;
pg.FaceColor = [0.3290 0.2 0.1250];
pg.LineWidth = 3;
pg.FaceAlpha = 0.3;
P = {pp, pc, pg};
%hold off
end