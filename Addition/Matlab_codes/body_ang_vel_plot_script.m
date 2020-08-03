for i = 1:3
  tmp(i) = subplot(3,1,i);  
  plot(x, body_ang_vel(i,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, body_ang_vel_des(i, 1:min_length), 'r-', 'linewidth',1.0);
  phase_drawing_script;
end
title(tmp(1), 'Body ang vel');
xlabel('Time (sec)','fontsize', 12);
