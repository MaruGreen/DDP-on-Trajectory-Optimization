
clear all
close all

%% Initial the time zone
time_step = 0.1;
duration = 8;
point_num = duration / time_step;
t = 0 : time_step : duration;

%% Calculate the spline point for initial trajectory
base = [36; -9];
spline_point = base * ones(1,9);
n = 12 : -1 : 4;
spline_point = spline_point + 7 * [cos(pi/24*n); sin(pi/24*n)];
spline_point(2,1) = spline_point(2,1) + 2;
figure,
plot(spline_point(1,:), spline_point(2,:), '-*r')
hold on,

%% Trick: spline to replace B-spline
spline_point = base * ones(1,9);
paul = [9 7.5 7.1 7 6.95 6.9 6.9 6.95 7];
spline_point = spline_point + paul .* [cos(pi/24*n); sin(pi/24*n)];

%% Generate initial trajectory
traj_x = spline(0:8, spline_point(1,:), t);
traj_y = spline(0:8, spline_point(2,:), t);
plot(traj_x, traj_y, '*-g')
hold on






