rfoot_idx = 13;
lfoot_idx = 25;
%rfoot_pos_led = sense_led(rfoot_idx:(rfoot_idx+2),:) - sense_led(lfoot_idx:(lfoot_idx+2),:);
%rfoot_pos_led(2,:) = rfoot_pos_led(2,:) + 0.0365;
rfoot_pos_led = raw_led(rfoot_idx:(rfoot_idx+2),:) - raw_led(lfoot_idx:(lfoot_idx+2),:);
rfoot_pos_led = rfoot_pos_led*0.001;
rfoot_pos_led(2,:) = rfoot_pos_led(2,:) + 0.0365;

for i = 1:3
  tmp(i) = subplot(3,1,i);  
  plot(x, rfoot_pos(i,1:min_length) ,'b-', 'linewidth',1.0);
  hold on
  plot(x, rfoot_pos_des(i, 1:min_length), 'r-', 'linewidth',1.0);
  plot(x, rfoot_pos_led(i, 1:min_length), 'c-', 'linewidth', 1.);
  phase_drawing_script;
end
title(tmp(1), 'Rfoot (right)');
xlabel('Time (sec)','fontsize', 12);
