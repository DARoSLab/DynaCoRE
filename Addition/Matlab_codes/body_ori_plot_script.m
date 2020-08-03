for i = 1:4
  tmp(i) = subplot(4,1,i);  
  plot(x, body_ori(i,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, body_ori_des(i, 1:min_length), 'r-', 'linewidth',1.0);
  if i == 1
    ylim([0.85, 1.05]);
  end
  phase_drawing_script;
end
title(tmp(1), 'Body ori');
xlabel('Time (sec)','fontsize', 12);


%for i = 1:3
%  tmp(i) = subplot(3,1,i);  
%  plot(x, body_ori_rpy(i,1:min_length) ,'b-', 'linewidth',1.0);
% % ylim([-0.2, 0.2])
%  phase_drawing_script;
%end
%title(tmp(1), 'Body ori');
%xlabel('Time (sec)','fontsize', 12);
