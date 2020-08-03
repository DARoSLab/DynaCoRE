t = fn_read_file(data_path, 'time', 1);

phase = fn_read_file(data_path, 'phase', 1);

torque  = fn_read_file(data_path, 'torque',6);
torque_cmd = fn_read_file(data_path, 'command', 6);
jpos_des = fn_read_file(data_path, 'jpos_des', 6);
jvel_des = fn_read_file(data_path, 'jvel_des', 6);
q   = fn_read_file(data_path, 'config', 13);
qdot = fn_read_file(data_path, 'qdot', 12);

kin_led = fn_read_file(data_path, 'LED_Kin_Pos', 33);
sense_led = fn_read_file(data_path, 'LED_Pos', 33);
raw_led = fn_read_file(data_path, 'LED_Pos_Raw',33);

reaction_force = fn_read_file(data_path, 'reaction_force', 6);
com_pos = fn_read_file(data_path, 'com_pos', 3);
com_vel = fn_read_file(data_path, 'com_vel', 3);
global_pos_local = fn_read_file(data_path, 'global_pos_local', 3);

est_com_state = fn_read_file(data_path, 'estimated_com_state', 4);

body_ori = fn_read_file(data_path, 'body_ori', 4);
body_ori_rpy = fn_read_file(data_path, 'body_ori_rpy', 3);
body_ang_vel = fn_read_file(data_path, 'body_ang_vel', 3);
com_pos_des = fn_read_file(data_path, 'com_pos_des', 3);
com_vel_des = fn_read_file(data_path, 'com_vel_des', 3);
body_ori_des = fn_read_file(data_path, 'body_ori_des', 4);
body_ang_vel_des = fn_read_file(data_path, 'body_ang_vel_des', 3);

rfoot_pos = fn_read_file(data_path, 'rfoot_pos', 3);
rfoot_vel = fn_read_file(data_path, 'rfoot_vel', 3);
lfoot_pos = fn_read_file(data_path, 'lfoot_pos', 3);
lfoot_vel = fn_read_file(data_path, 'lfoot_vel', 3);

rfoot_pos_des = fn_read_file(data_path, 'rfoot_pos_des',3);
rfoot_vel_des = fn_read_file(data_path, 'rfoot_vel_des',3);
lfoot_vel_des = fn_read_file(data_path, 'lfoot_vel_des',3);
lfoot_pos_des = fn_read_file(data_path, 'lfoot_pos_des',3);
