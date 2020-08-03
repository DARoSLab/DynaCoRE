for i = 1:3
  tmp(i) = subplot(3,1,i);  
  plot(x, com_pos(i,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, com_pos_des(i, 1:min_length), 'r-', 'linewidth',1.0);
  if(i ~= 3)
    plot(x, est_com_state(i, 1:min_length), 'c-', 'linewidth',1.0);
  end
  phase_drawing_script;
  if i == 3
  ylim([0.6, 1.0]);
  end
end
title(tmp(1), 'Com Pos');
xlabel('Time (sec)','fontsize', 12);
