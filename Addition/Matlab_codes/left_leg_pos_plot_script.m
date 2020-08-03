rfoot_idx = 13;
lfoot_idx = 25;
%lfoot_pos_led = sense_led(lfoot_idx:(lfoot_idx+2),:) - sense_led(rfoot_idx:(rfoot_idx+2),:);
%lfoot_pos_led(2,:) = lfoot_pos_led(2,:) - 0.0365;

lfoot_pos_led = raw_led(lfoot_idx:(lfoot_idx+2),:) - raw_led(rfoot_idx:(rfoot_idx+2),:);
lfoot_pos_led = lfoot_pos_led * 0.001;
lfoot_pos_led(2,:) = lfoot_pos_led(2,:) - 0.0365;


for i = 1:3
  tmp(i) = subplot(3,1,i);  
  plot(x, lfoot_pos(i,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, lfoot_pos_des(i, 1:min_length), 'r-', 'linewidth',1.0);
  plot(x, lfoot_pos_led(i, 1:min_length), 'c-', 'linewidth', 1.0);
  phase_drawing_script;
end
title(tmp(1), 'Lfoot pos');
xlabel('Time (sec)','fontsize', 12);
