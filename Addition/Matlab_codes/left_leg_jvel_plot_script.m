for i = 1:3
  tmp(i) = subplot(3,1,i);  
  plot(x, qdot(i+9,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, jvel_des(i+3, 1:min_length), 'r-', 'linewidth',1.0);
  phase_drawing_script;
end
title(tmp(1), 'Jvel (left)');
xlabel('Time (sec)','fontsize', 12);
