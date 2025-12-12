function h = circle(x,y,r)
%x, y center
% radius
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
%h = plot(xunit, yunit);
fill(xunit,yunit,'green', 'FaceAlpha', 0.1, 'LineWidth',0.01, edgeColor = 'none')
hold off
end