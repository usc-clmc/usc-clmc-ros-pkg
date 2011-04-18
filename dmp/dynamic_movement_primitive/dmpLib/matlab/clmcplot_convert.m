function [D,vars,freq] = clmcplot_convert(fname)
% [D,vars,freq] = clmcplot_convert(fname)
%
% This function converts an CLMCPLOT binary file into a Matlab matrix and
% a struct array of variable names and variable units. If fname is
% given, the file is processed immediately. If no filename is given,
% a dialog box will ask to located the file.
%
% fname (i): input file name (optional)
% D     (o): data matrix
% vars  (o): struct array containing variable names and units
% freq  (o): sampling frequency
%

% Stefan Schaal, March 2006

% read in the file name
if ~exist('fname') | isempty(fname),
	[fname, pathname] = uigetfile('*','Select Data File');
	if (fname == 0),
		return;
	end;
	% concatenate pathname and filename and open file
	fname_store = fname;
	fname=strcat(pathname, fname);
end;

fid=fopen(fname, 'r', 'ieee-be');
if fid == -1,
	return;
end;

% check for BDIPLOT file format
string = fscanf(fid,'%s',1);
frewind(fid);
if strcmp(string,'$BEGIN_HEADER'),  % this is a BDIPLOT file

  vars = [];

  while (1),

    string=fscanf(fid,'%s',1);

    if (strcmp(string,'$DT')),
      dt=fscanf(fid,'%f',1);
      freq = 1/dt;
    end

    if (strcmp(string,'$NVAR')),
      cols=fscanf(fid,'%d',1);
    end

    if (strcmp(string,'$VAR')),
      vars(end+1).name = fscanf(fid,'%s',1);
      vars(end).unit   = '-';
    end

    if (strcmp(string,'$OUT')),
      rows=fscanf(fid,'%d',1);
    end

    if (strcmp(string,'$END_HEADER')),
      break;
    end

  end

  % read the data
  fscanf(fid,'%c',1);

else                                % this is a CLMCPLOT file

  specs=fscanf(fid,'%d %d %d %f',4);  % [dummy,cols,rows,freq]
  cols = specs(2);
  rows = specs(3);
  freq = specs(4);
  vars = [];

  % read all variable names
  for i=1:cols,
    vars(i).name=fscanf(fid,'%s',1);
    vars(i).unit=fscanf(fid,'%s',1);
  end;
  fscanf(fid,'%c',3); % there are three characters which must be skipped

end

% read the data
D = fread(fid, [cols,rows],'float32');
D=D';
fclose(fid);
