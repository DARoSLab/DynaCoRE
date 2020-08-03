clear all
clc 
close all

%% 
fn_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
addpath(fn_path)
data_path = '/home/hcrl/MyCloud/Apptronik/HumeSingleLeg/experiment_data_check';
%data_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/experiment_data_check';

read_data_simple_file;
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

NUM_STEP = 3;
fig = fn_open_figures(NUM_STEP);

%% Find start index of phase 3 and end index of phase 8
third_ini_idx = [];
eighth_fin_idx = [];
third_idx = find(phase == 3);
eighth_idx = find(phase == 8);

j=1;
third_ini_idx(j) = third_idx(1);
j=j+1;
for i = 2:length(third_idx)
  if third_idx(i-1) + 1 ~= third_idx(i)
    third_ini_idx(j) = third_idx(i);
    j = j+1;
  end
end
j=1;
for i = 1:length(eighth_idx)-1
  if eighth_idx(i) + 1 ~= eighth_idx(i+1)
    eighth_fin_idx(j) = eighth_idx(i);
    j = j+1; 
 end
end
eighth_fin_idx(j) = eighth_idx(end);
if length(third_ini_idx) ~= length(eighth_fin_idx)
  'wrong parsing'
end
%% Draw Figure
for i = 1:NUM_STEP
  figure(fig(i))
  ini = third_ini_idx(i);
  fin = eighth_fin_idx(i);
  com_phase_plot_script;
end