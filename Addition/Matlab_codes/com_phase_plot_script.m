%% CoM local to global

com_pos(1:2,:) = est_com_state(1:2,:);
com_vel(1:2,:) = est_com_state(3:4,:);
com_pos_glob = global_pos_local + com_pos;
com_vel_glob = com_vel;
com_pos_glob_des = global_pos_local + com_pos_des;
com_vel_glob_des = com_vel_des;

for i = 1:2
  tmp(i) = subplot(2,1,i);  
%  plot(com_pos_glob(i,ini:fin), com_vel_glob(i,ini:fin) ,'b-', 'linewidth',1.0);
  plot(com_pos_glob(i,ini:fin), com_vel_glob(i,ini:fin), '*');

  hold on
%  plot(com_pos_glob_des(i,ini:fin), com_vel_glob_des(i, ini:fin), 'r-', 'linewidth',1.0);
plot(com_pos_glob_des(i,ini:fin), com_vel_glob_des(i, ini:fin),'o');
end
title(tmp(1), 'COM Phase Plot');