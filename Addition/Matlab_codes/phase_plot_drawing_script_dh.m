curr_st_idx = phase_swing_idx(step_num);
curr_sense_end_idx = phase_swing_idx(step_num+1);
curr_end_idx = phase_swing_end_idx(step_num);
nx_st_idx = phase_swing_idx(step_num+1);
nx_end_idx = phase_swing_end_idx(step_num+1);

%%%%% Draw the planned path
ini_com_state = [curr_com_state(plot_dir,step_num), curr_com_state(plot_dir + 2, step_num)];
this_swing_time = swing_time(step_num);
num_dots = 200;
planned_com_path = zeros(2, num_dots);
dt = this_swing_time/num_dots;
stance_foot_loc = global_pos_local(plot_dir, curr_st_idx);

A = ((ini_com_state(1) - stance_foot_loc) + ini_com_state(2)/omega)/2.;
B = ((ini_com_state(1) - stance_foot_loc) - ini_com_state(2)/omega)/2.;

for i = 1:num_dots
    planned_com_path(1,i) = A * exp(omega * dt*(i-1)) + B * exp(-omega * dt*(i-1)) + stance_foot_loc;
    planned_com_path(2,i) = omega * (A * exp(omega * dt*(i-1)) - B * exp(-omega * dt*(i-1)));
end

%%%%% Draw the next planned path
nx_ini_com_state = [switch_com_state(plot_dir,step_num), switch_com_state(plot_dir + 2, step_num)];
next_swing_time = 0.3;
nx_planned_com_path = zeros(2, num_dots);
dt_nx = next_swing_time/num_dots;
nx_stance_foot_loc = nx_planned_foot(plot_dir, step_num);

A_nx = ((nx_ini_com_state(1) - nx_stance_foot_loc) + nx_ini_com_state(2)/omega)/2.;
B_nx = ((nx_ini_com_state(1) - nx_stance_foot_loc) - nx_ini_com_state(2)/omega)/2.;

for i = 1:num_dots
    nx_planned_com_path(1,i) = A_nx * exp(omega * dt_nx*(i-1)) + B_nx * exp(-omega * dt_nx*(i-1)) + nx_stance_foot_loc;
    nx_planned_com_path(2,i) = omega * (A_nx * exp(omega * dt_nx*(i-1)) - B_nx * exp(-omega * dt_nx*(i-1)));
end


%%%%%%%%%%% Drawing
hold on
% curr sensed
plot(com_pos(plot_dir, curr_st_idx: curr_sense_end_idx), com_vel(plot_dir, curr_st_idx:curr_sense_end_idx), 'b-');
% curr estimated
plot(est_com_state(plot_dir, curr_st_idx: curr_end_idx), est_com_state(plot_dir+2, curr_st_idx:curr_end_idx), 'r-');
% curr planned
plot(planned_com_path(1,:), planned_com_path(2,:), 'k-');

% picked state
plot(curr_com_state(plot_dir, step_num), curr_com_state(plot_dir + 2, step_num),'r*', 'markersize',7);
% expected switching state
plot(switch_com_state(plot_dir, step_num), switch_com_state(plot_dir + 2, step_num),'c*', 'markersize',7);
% planned step
plot(nx_planned_foot(plot_dir, step_num), 0,'ro', 'markersize',10);
% actual nx step
plot(global_pos_local(plot_dir, phase_swing_idx(step_num+1)), 0,'gx', 'markersize',12);

% current stance
plot(global_pos_local(plot_dir, phase_swing_idx(step_num)), 0,'b+', 'markersize',12);

% next
plot(com_pos(plot_dir, nx_st_idx: nx_end_idx), com_vel(plot_dir, nx_st_idx:nx_end_idx), 'g-');
plot(est_com_state(plot_dir, nx_st_idx: nx_end_idx), est_com_state(plot_dir+2, nx_st_idx:nx_end_idx), 'c-');

% next planned
plot(nx_planned_com_path(1,:), nx_planned_com_path(2,:), 'm-');

%% curr
%plot(com_pos(plot_dir, curr_st_idx: curr_end_idx), com_vel(plot_dir, curr_st_idx:curr_end_idx), 'b*');
%plot(est_com_state(plot_dir, curr_st_idx: curr_end_idx), est_com_state(plot_dir+2, curr_st_idx:curr_end_idx), 'r*');
%% next
%plot(com_pos(plot_dir, nx_st_idx: nx_end_idx), com_vel(plot_dir, nx_st_idx:nx_end_idx), 'g.');
%plot(est_com_state(plot_dir, nx_st_idx: nx_end_idx), est_com_state(plot_dir+2, nx_st_idx:nx_end_idx), 'c.');

title(step_num)
if plot_dir == 1
xlabel('x');
else 
xlabel('y');
end