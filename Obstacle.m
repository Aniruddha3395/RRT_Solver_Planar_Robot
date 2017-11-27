function [obst_plot] = Obstacle(C,R)

% defining objects in workspace

%circle generation polyline
I = [0:0.1:2*pi]';
x = R*cos(I)+C(1);
x = [x;x(1,:)];
y = R*sin(I)+C(2);
y = [y;y(1,:)];

obst_plot = plot(x,y);
fill(x,y,'g')
xlim([-1000,1000]);
ylim([0,1000]);
end