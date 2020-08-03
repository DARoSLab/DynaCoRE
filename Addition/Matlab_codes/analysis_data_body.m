clear all
clc 
close all

%% 
%fn_path = '/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
fn_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';

addpath(fn_path)

%data_path = '/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit/experiment_data_check';
data_path = '/home/hcrl/MyCloud/Apptronik/MercuryTest/experiment_data_check';
read_data_simple_file;

fig = fn_open_figures(10);

min_length = length(t) - 0;
x = t(1:min_length);

j = 1;
phase_change_idx = [];
for i = 1:min_length-1
    if(phase(i+1) ~= phase(i))
        phase_change_idx(j) = i;    
        j = j+1;
    end    
end
%% Draw Figure
% Torque
figure(fig(1))
right_leg_torque_plot_script;
figure(fig(2))
left_leg_torque_plot_script;

%% joint pos
figure(fig(3))
right_leg_jpos_plot_script;

figure(fig(4))
right_leg_jvel_plot_script;

figure(fig(5))
left_leg_jpos_plot_script;

figure(fig(6))
left_leg_jvel_plot_script;

figure(fig(7))
com_pos_plot_script;

figure(fig(8))
com_vel_plot_script;

figure(fig(9))
body_ori_plot_script;

figure(fig(10))
body_ang_vel_plot_script;