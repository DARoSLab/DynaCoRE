for i = 1:3
  tmp(i) = subplot(3,1,i);  
  plot(x, lfoot_vel(i,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, lfoot_vel_des(i, 1:min_length), 'r-', 'linewidth',1.0);
  phase_drawing_script;
end
title(tmp(1), 'Lfoot vel');
xlabel('Time (sec)','fontsize', 12);
