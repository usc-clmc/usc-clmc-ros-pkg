clear all;
close all;

base_file_name='/tmp/dmp_waypoint_task_noisless_rollout_';
% base_file_name='/tmp/dmp_waypoint_task_noisy_rollout_';

figure(1);

for i=0:99
  fname = sprintf('%s%d.clmc',base_file_name,i);
  [data, vars, freq] = clmcplot_convert(fname);
  
  time = linspace(0,1,length(data(:,1)));

  clf;
  hold on;
  plot(time, data(:,1));
  plot(0.5, 2.5, 'go', 'linewidth', 2.0);
  hold on;
  box on;
  axis([0, 1, -0.5, 4.0]);

  pause(0.02);
end
