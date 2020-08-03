clear all
clc 
close all

%% 
fn_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';

addpath(fn_path)

data_path = '/home/hcrl/MyCloud/Apptronik/MercuryTest/experiment_data_check';
%read_data_simple_file;
a1 = fn_read_file(data_path, '1_lg1j2_mapped__motor__input', 1);
a2 = fn_read_file(data_path, '2_lg1j1_mapped__motor__input', 1);
a3 = fn_read_file(data_path, '3_lg1j0_mapped__motor__input', 1);
a4 = fn_read_file(data_path, '4_lg0j0_mapped__motor__input', 1);
a5 = fn_read_file(data_path, '5_lg0j1_mapped__motor__input', 1);
a6 = fn_read_file(data_path, '6_lg0j2_mapped__motor__input', 1);

fig = fn_open_figures(6);

figure(fig(1))
plot(a1);

figure(fig(2))
plot(a2);

figure(fig(3))
plot(a3);

figure(fig(4))
plot(a4);

figure(fig(5))
plot(a5);

figure(fig(6))
plot(a6);