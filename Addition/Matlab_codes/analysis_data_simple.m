clear all
clc 
close all

%% 
fn_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
addpath(fn_path)

data_path = '/home/hcrl/ros/apptronik_two_dt/data_manager/experiment_data_check';
read_data_simple_file;
fig = fn_open_figures(6);

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

%% Reaction Force

figure(fig(3))
%right_leg_reaction_force_plot_script;
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(x, rfoot_pos_des(i,:), 'linewidth',3);
    plot(x, rfoot_pos(i,:));
  phase_drawing_script;

end
figure(fig(4))
%left_leg_reaction_force_plot_script;
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(x, lfoot_pos_des(i,:), 'linewidth',3);
    plot(x, lfoot_pos(i,:));
      phase_drawing_script;

end

figure(fig(5))
%right_leg_reaction_force_plot_script;
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(x, rfoot_vel_des(i,:), 'linewidth',3);
    plot(x, rfoot_vel(i,:));
  phase_drawing_script;

end
figure(fig(6))
%left_leg_reaction_force_plot_script;
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(x, lfoot_vel_des(i,:), 'linewidth',3);
    plot(x, lfoot_vel(i,:));
      phase_drawing_script;

end