function gen_clmc_file_from_ascii(rec_fname, clmc_fname)

  X  = load(rec_fname,'-ascii');
  
  if(size(X, 2) ~= 3)
     error('Number of trajectories should equal 3 (time, x, y)');     
  end
  
  n  = length(X);
  X  = [(0:n-1+10)'*1/60 [ones(5,1)*X(1,2);X(:,2);ones(5,1)*X(end,2)]  [ones(5,1)*X(1,3);X(:,3);ones(5,1)*X(end,3)]];

  [b,a]=butter(2,0.2);
  X(:,2) = filtfilt(b,a,X(:,2));
  X(:,3) = filtfilt(b,a,X(:,3));
  
  dt = 1/60;
  
  vars(1).name = 'time';
  vars(2).name = 'R_HAND_des_x';
  vars(3).name = 'R_HAND_des_y';
   
  vars(1).unit = 'sec';
  vars(2).unit = 'm';
  vars(3).unit = 'm';
  
  clmcplot_gen(X, vars, 1/dt, clmc_fname);