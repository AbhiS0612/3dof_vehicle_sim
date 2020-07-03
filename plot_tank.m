function p = plot_tank()
%%plot_tank() Plot the Hydrolab tesk tank.
r = 3.3;    %tank radius in meters
theta = 0:pi/50:2*pi;
x_circle = r * cos(theta);
y_circle = r * sin(theta);
light_blue = [.5, .7, 1];
p = fill(x_circle, y_circle, light_blue);
p.LineWidth = 2;
p.FaceAlpha = 0.4;
axis equal
grid on;
end