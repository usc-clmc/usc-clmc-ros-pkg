function clmcplot_gen(data,vars,freq,fname)
% clmcplot_gen(data,vars,freq,fname)
%
% This function writes a data matrix into CLMCPLOT binary file. The variable
% names and units are given in a struct array vars, having vars.name and
% vars.unit for every column. 
%
% data  (i): the data matrix to be written
% vars  (i): names and units of the data columns as a struct array
% freq  (i): sampling frequency of the data
% fname (i): filename of output file (optional)
%
% Stefan Schaal, March 2006

if ~exist('fname') | isempty(fname),
	[fname, pathname] = uiputfile('d*','Save under which filename?');
	if (fname == 0),
		return;
	end;

	% concatenate pathname and filename and open file
	fname_store = fname;
	fname=strcat(pathname, fname);
end

% open the file
fid=fopen(fname, 'w','ieee-be');
if fid == -1,
  return;
end;

% write the variables
[rows,cols]=size(data);
fprintf(fid,'%d %d %d %d',cols*rows,cols,rows,freq);

% write all variable names
for i=1:cols,
	fprintf(fid,'%s  ',vars(i).name);
	fprintf(fid,'%s  ',vars(i).unit);
end;
fprintf(fid,'\n');

% write the data
data = fwrite(fid,data','float32');
fclose(fid);
