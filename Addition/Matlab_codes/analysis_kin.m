clear all
clc 
close all

%% 
%fn_path = '/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
fn_path = '/home/hcrl/Repository/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
addpath(fn_path)
%data_path = '/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit/experiment_data_check';
data_path = '/home/hcrl/MyCloud/Apptronik/MercuryTest/experiment_data_check';

rbdl = fn_read_file(data_path, 'LED_Kin_Pos', 33);
led = fn_read_file(data_path, 'LED_Pos', 33);

NUM_MARKER = 11;
fig = fn_open_figures(NUM_MARKER + 3);

%%% Draw Figure
for i = 1 : NUM_MARKER
figure(fig(i))
for j = 1 : 3
subplot(3, 1, j)
plot(led(3*(i-1) + j, :),'r')
hold on
plot(rbdl(3*(i-1) + j, :),'b')
hold off
end
end
body0 = 0;
leg0 = 7;
leg1 = 8;
body1 = 9;
body2 = 10;
start_idx = 500;32
end_idx = length(led) - 600;

%% Figure (X, Z)
figure(fig(NUM_MARKER+1))
hold on
plot(led(3*leg0+1,start_idx:end_idx), led(3*leg0+3,start_idx:end_idx), 'r*');
plot(rbdl(3*leg0+1,start_idx:end_idx), rbdl(3*leg0+3,start_idx:end_idx), 'k*');
plot(led(3*leg1+1,start_idx:end_idx), led(3*leg1+3,start_idx:end_idx), 'b*');
plot(rbdl(3*leg1+1,start_idx:end_idx), rbdl(3*leg1+3,start_idx:end_idx), 'g*');
x = linspace(0.01, 0.4, 100);
y = -tan(0.05)*(x-0.13)-0.136;
plot(x, y,'c')


%% Figure (X, Y)
figure(fig(NUM_MARKER+2))
hold on
plot(led(3*leg0+1,start_idx:end_idx), led(3*leg0+2,start_idx:end_idx), 'r*');
plot(rbdl(3*leg0+1,start_idx:end_idx), rbdl(3*leg0+2,start_idx:end_idx), 'k*');
plot(led(3*leg1+1,start_idx:end_idx), led(3*leg1+2,start_idx:end_idx), 'b*');
plot(rbdl(3*leg1+1,start_idx:end_idx), rbdl(3*leg1+2,start_idx:end_idx), 'g*');
x = linspace(0.01, 0.4, 100);
y = tan(-0.02)*(x-0.13)-0.136;
plot(x, y,'c')

%% Figure (Y, Z)
figure(fig(NUM_MARKER+3))
hold on
plot(led(3*leg0+2,start_idx:end_idx), led(3*leg0+3,start_idx:end_idx), 'r*');
plot(rbdl(3*leg0+2,start_idx:end_idx), rbdl(3*leg0+3,start_idx:end_idx), 'k*');
plot(led(3*leg1+2,start_idx:end_idx), led(3*leg1+3,start_idx:end_idx), 'b*');
plot(rbdl(3*leg1+2,start_idx:end_idx), rbdl(3*leg1+3,start_idx:end_idx), 'g*');
x = linspace(-0.15, -0.12, 100);
y = -tan(pi/2 - 0.008)*(x+0.135)+0.02;
plot(x, y,'c')