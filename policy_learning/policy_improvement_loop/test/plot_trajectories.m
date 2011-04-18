base_file_name='/tmp/ctp_';

figure(1);
for i=1:100
  file_name = sprintf('%s%d.txt',base_file_name,i);
  data=load(file_name);
  plot(data);
  pause(0.1);
end
