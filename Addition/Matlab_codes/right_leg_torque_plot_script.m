for i = 1:3
  tmp(i) = subplot(3,1,i);  
  plot(x, torque(i,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, torque_cmd(i, 1:min_length), 'r-', 'linewidth',1.0);
  phase_drawing_script;
end
legend('sensed','cmd');
title(tmp(1), 'Torque (right)');
xlabel('Time (sec)','fontsize', 12);
