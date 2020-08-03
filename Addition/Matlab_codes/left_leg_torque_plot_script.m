%a2 = fn_read_file(data_path, '2_lg1j1_mapped__motor__input', 1);

for i = 1:3
  tmp(i) = subplot(3,1,i);
  plot(x, torque(i+3,1:min_length) ,'b-', 'linewidth',1.2);
  hold on
    plot(x, torque_cmd(i+3, 1:min_length), 'r-', 'linewidth',1.0);
end

xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'Torque (left)');
