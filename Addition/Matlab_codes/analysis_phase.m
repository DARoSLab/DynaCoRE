clear all
clc 
close all

%% 
%fn_path = '/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
%fn_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
fn_path = '/Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';

addpath(fn_path)

%data_path = '/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit/experiment_data_check';
%data_path = '/home/hcrl/MyCloud/Apptronik/MercuryTest/experiment_data_check';
data_path = '/Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/experiment_data_check';
%data_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/experiment_data_check';

%% read files
t = fn_read_file(data_path, 'time', 1);
phase = fn_read_file(data_path, 'phase', 1);

kin_led = fn_read_file(data_path, 'LED_Kin_Pos', 33);
sense_led = fn_read_file(data_path, 'LED_Pos', 33);

global_pos_local = fn_read_file(data_path, 'global_pos_local', 3);
com_pos = fn_read_file(data_path, 'com_pos', 3);
com_vel = fn_read_file(data_path, 'com_vel', 3);
est_com_state = fn_read_file(data_path, 'estimated_com_state', 4);

com_pos(1:2,:) = com_pos(1:2, :) + global_pos_local(1:2, :);
est_com_state(1:2,:) = est_com_state(1:2,:) + global_pos_local(1:2,:);

com_pos_des = fn_read_file(data_path, 'com_pos_des', 3);
com_vel_des = fn_read_file(data_path, 'com_vel_des', 3);

rfoot_pos = fn_read_file(data_path, 'rfoot_pos', 3);
rfoot_vel = fn_read_file(data_path, 'rfoot_vel', 3);
lfoot_pos = fn_read_file(data_path, 'lfoot_pos', 3);
lfoot_vel = fn_read_file(data_path, 'lfoot_vel', 3);

rfoot_pos_des = fn_read_file(data_path, 'rfoot_pos_des',3);
rfoot_vel_des = fn_read_file(data_path, 'rfoot_vel_des',3);
lfoot_vel_des = fn_read_file(data_path, 'lfoot_vel_des',3);
lfoot_pos_des = fn_read_file(data_path, 'lfoot_pos_des',3);

planner_data = fn_read_file(data_path, 'planner_data', 11);


st_idx = 1;
min_length = length(t) - 100;
x = t(st_idx:min_length);

j = 1;
k = 1;
phase_swing_idx = [];
phase_swing_end_idx = [];
i = 1;
while(i < min_length)
  % right swing start
  if(phase(i) == 4)
    phase_swing_idx(j) = i;     
    while(phase(i) == 4)      
        i = i +1;
    end
    j = j+1;
  end    
  % right swing end
  if(phase(i) == 5)
    phase_swing_end_idx(k) = i;     
    while(phase(i) == 5)      
        i = i +1;
    end
    k = k+1;
  end 
  
  % left swing start
  if(phase(i) == 8)
    phase_swing_idx(j) = i;
    while(phase(i) == 8)      
      i = i +1;
    end
    j = j+1;
  end    
  % left swing end
  if(phase(i) == 9)
    phase_swing_end_idx(k) = i;
    while(phase(i) == 9)      
      i = i +1;
    end
    k = k+1;
  end
  i = i+1;
end
omega = sqrt(9.81/com_pos_des(3,phase_swing_idx(2)));

curr_com_state = planner_data(1:4,:);
switch_com_state = planner_data(5:8,:);
nx_planned_foot = planner_data(9:10,:);
swing_time = planner_data(11,:);


%% Draw Figure
number_steps = 3;
fig = fn_open_figures(2*number_steps);

for step_num = 1:number_steps
    figure(fig(2*step_num - 1))
    plot_dir = 1;
    phase_plot_drawing_script_dh;

    figure(fig(2*step_num))
    plot_dir = 2;
    phase_plot_drawing_script_dh;
end
