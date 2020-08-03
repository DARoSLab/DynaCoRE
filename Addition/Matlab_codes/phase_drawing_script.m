   ylim_plot = get(gca, 'YLim');
   for k = 1:length(phase_change_idx)
     phase_idx = phase_change_idx(k);
     line([x(phase_idx), x(phase_idx)], ylim_plot);
   end
