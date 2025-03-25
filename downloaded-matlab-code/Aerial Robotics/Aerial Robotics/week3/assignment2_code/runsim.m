clear;
close all;

addpath('utils');
addpath('trajectories');

controlhandle = @controller;

% Choose which trajectory you want to test with
% trajhandle = @traj_line;
 trajhandle = @traj_sine;

[t, state] = simulation_2d(controlhandle, trajhandle);

% --- Temp test code from assignment 1---
% Sample code to get more info on the response
% sim_info = lsiminfo(z,t,z_des);
% disp(['Settling time [s]: ', num2str(sim_info.SettlingTime)]);
% disp(['Overshoot [%]: ', num2str(max(0,(sim_info.Max-z_des)*100))]);
% Temp test code ends
