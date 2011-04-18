function clmcplot_functions(task)
% this part of the program implements the switch to various functiontalities
% of CLMCPLOT
% Copyright 2008 Stefan Schaal
% Computational Learning and Motor Control Laboratory
% University of Southern California
% December 1997-2008


switch task,
  
case 'init',
	init_clmcplot;
 case 'open',
	clmcplot_open(0);
case 'fast_open',
	clmcplot_open(1);
case 'zoom',
	clmcplot_zoom;
case 'cursor',
	clmcplot_cursor;
case 'clear_all',
	clear_all;
case 'key_press',
	key_press;
case 'varlist',
	clmcplot_varlist;
case 'add_subplot',
	add_subplot(1);
case 'delete_subplot',
	delete_subplot(1);
case 'save_view',
	save_view;
case 'viewlist',
	clmcplot_viewlist;
case 'toggle_legend',
	clmcplot_toggle_legend;

otherwise,
	return;  
end;


%----------------------------------------------------------------------------------------------
% the function that writes the current data point as a clmcplot file
function write_datapoint()

global MRDS;

if ~isempty(MRDS.data)
  fp = fopen('.tmp_clmcplot','w');
  fprintf(fp,'%s %d',MRDS.fname,MRDS.cursor);
  fclose(fp);
  movefile('.tmp_clmcplot','.clmcplot_current_point');
end

%----------------------------------------------------------------------------------------------
% the function that initalizes CLMCPLOT
function init_clmcplot()

global MRDS;  % the structure containing all the data for CLMCPLOT
% all pointers are stored in the MRDS structure, which is global and will thus
% not be forgotten


% some useful initializations
MRDS.n_subplots     = 6;
MRDS.rows           = 1;
MRDS.cols           = 1;
MRDS.freq           = 1;
MRDS.zoom           = 20;
MRDS.cursor         = 1;
MRDS.time           = toc;
MRDS.flag           = 0;
MRDS.var1 = 0;
MRDS.var2 = 0;
MRDS.string         = [];
MRDS.varnames       = {};
MRDS.data           = [];
MRDS.fontsize       = 9;
MRDS.fname          = [];
MRDS.views          = default_views;
MRDS.legend_pos     = 'NorthEast';

if exist('.clmcplot') == 2,
  load('.clmcplot','-mat');
  MRDS.views = views;
end


% generate the command and the data window
clmcplot_command;
clmcplot_data;
rebuild_view_list(1);

return;
%----------------------------------------------------------------------------------------------
% creates the CLMCPLOT command window
function clmcplot_command()

global MRDS;
bw = 72; % button width
bh = 23; % button height
bs = 6;  % button spacing
bb = 11;  % button begin

scrollbar_width=25;
if (size(MRDS.screensize,1) > 1)
  fig_size = MRDS.screensize(2,:);
else
  s = MRDS.screensize;
  fig_size(1) = s(1)+s(3)-260-scrollbar_width+1;
  fig_size(2) = s(2)+50;
  fig_size(3) = 250;
  fig_size(4) = s(4)-50-(fig_size(2)-s(2));
end

if ishandle(450),
  delete(450);
end;  % a pre-caution

ptr = figure(450);  % the window number is arbitrarily set to 450
set(ptr,...
'Units','Pixels', ...	
'Color',[0.8 0.8 0.8], ...
'NumberTitle','off', ...
'Position',fig_size, ...
'Name','CLMCPLOT-Command', ...
'Resize','off',...
'Position',fig_size, ...
'MenuBar','none',...
'Tag','Fig3');

% the pointer to the command window is stored in the global structure
MRDS.cptr = ptr; 

% create the list for the variables
lptr = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''varlist'');',...
'BackgroundColor',[1 1 1], ...
'Position',[10 295 fig_size(3)-20 floor(fig_size(4))-20-195-100], ...
'Style','listbox', ...
'Tag','List of Variables', ...
'FontSize',9,...
'FontName','Monaco',...
'SelectionHighlight','off',...
'Value',1);

% the list pointer is also kept in the global structure a header for the list of
% variables
MRDS.lptr = lptr;  


b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'BackgroundColor',[1 0.5 0], ...
'Position',[10 fig_size(4)-18 fig_size(3)-20 16], ...
'String','Variables', ...
'Style','text', ...
'FontSize',14, ...
'Tag','Header of Variable List');

MRDS.nptr=b;


% create the list for the views
vptr = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''viewlist'');',...
'BackgroundColor',[1 1 1], ...
'Position',[10 195 fig_size(3)-20 80], ...
'Style','listbox', ...
'Tag','List of Views', ...
'FontSize',9,...
'FontName','Monaco',...
'SelectionHighlight','off',...
'Value',1);

MRDS.vptr = vptr;  

b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'BackgroundColor',[1 0.5 0], ...
'Position',[10 276 fig_size(3)-20 16], ...
'String','Views', ...
'Style','text', ...
'FontSize',14, ...
'Tag','Header of View List');


% display of time and tick
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'BackgroundColor',[1 1 1], ...
'Position',[10 3 fig_size(3)-20 10], ...
'String','time=0.0  tick=0', ...
'Style','text', ...
'Tag','Time and Ticks');
MRDS.tptr=b;

% the buttons of this window
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;axes(MRDS.axes(MRDS.n_subplots));title(sprintf(''%s (%s)'',MRDS.fname,datestr(now,0)));orient landscape; print -f451;',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb 167 bw bh], ...
'String','Print', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;axes(MRDS.axes(MRDS.n_subplots));title(sprintf(''%s (%s)'',MRDS.fname,datestr(now,0)));orient landscape; eval(sprintf(''print %s.plot -dmfile -f451'',MRDS.fname));',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[bb+bw+bs 167 bw bh], ...
'String','Save', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;MRDS.time=toc;MRDS.flag=7;MRDS.var1=0;MRDS.var2=0;',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[bb+(bw+bs)*2 167 bw bh], ...
'String','Add/Sub', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''open'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb 137 bw bh], ...
'String','Open ...', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;if MRDS.zoom<500,MRDS.zoom=MRDS.zoom*1.5;end;clmcplot_functions(''zoom'');',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[bb+bw+bs 137 bw bh], ...
'String','Zoom in', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;if MRDS.zoom>1,MRDS.zoom=MRDS.zoom/1.5;end;clmcplot_functions(''zoom'');',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[bb+(bw+bs)*2 137 bw bh], ...
'String','Zoom out', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''fast_open'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb 107 bw bh], ...
'String','Fast Open', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;MRDS.time=toc;MRDS.flag=4;MRDS.var1=0;MRDS.var2=0;',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[bb+bw+bs 107 bw bh], ...
'String','Phase Plot', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;MRDS.time=toc;MRDS.flag=6;',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[bb+(bw+bs)*2 107 bw bh], ...
'String','d/dt', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''toggle_legend'');',...
'BackgroundColor',[0.3 0.9 0.5], ...
'Position',[bb 77 bw bh], ...
'String','Legend', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''save_view'');',...
'BackgroundColor',[0.3 0.9 0.5], ...
'Position',[bb+bw+bs 77 bw bh], ...
'String','Save View', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;MRDS.time=toc;MRDS.flag=100;',...
'BackgroundColor',[0.3 0.9 0.5], ...
'Position',[bb+(bw+bs)*2 77 bw bh], ...
'String','Delete View', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;MRDS.time=toc;MRDS.flag=2;',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb 47 bw bh], ...
'String','Clear', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;MRDS.time=toc;MRDS.flag=3;',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb+bw+bs 47 bw bh], ...
'String','Clear Last', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''clear_all'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb+(bw+bs)*2 47 bw bh], ...
'String','Clear All', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''add_subplot'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb 17 bw bh], ...
'String','Add Plot', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','clmcplot_functions(''delete_subplot'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[bb+bw+bs 17 bw bh], ...
'String','Del Plot', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','pixels', ...
'Callback','global MRDS;delete([MRDS.cptr,MRDS.dptr]);clear MRDS;',...
'BackgroundColor',[0.9 0.1 0.1], ...
'Position',[bb+(bw+bs)*2 17 bw bh], ...
'String','Quit', ...
'Tag','Pushbutton1');

%-------------------------------------------------------------------------------------------
function clmcplot_open(flag)
% open and reads in a file

global MRDS;

% for flag==1, we try to read the filename from the file last_data

fname =[];

if exist('last_data'), % migration from last_data to .last_data
  unix('mv last_data .last_data');
end;

if exist('.last_data') && flag == 1,
	fid = fopen('.last_data','r');
	fileid = fscanf(fid,'%d');
	last_data = sprintf('d%05d',fileid-1);
	fclose(fid);
	if exist(last_data),
		fname = last_data;
		pathname=' ';
	end;
end;

% read in the file name
if isempty(fname),
	[fname, pathname] = uigetfile('*','Select Data File');
	if (fname == 0),
		return;
	end;
end;

% concatenate pathname and filename and open file
fname_store = fname;
fname=strcat(pathname, fname);
fid=fopen(fname, 'r','ieee-be');
if fid == -1,
	return;
end;

% check for BDIPLOT file format
string = fscanf(fid,'%s',1);
frewind(fid);
if strcmp(string,'$BEGIN_HEADER'),  % this is a BDIPLOT file

  MRDS.vars = [];

  while (1),

    string=fscanf(fid,'%s',1);

    if (strcmp(string,'$DT')),
      dt=fscanf(fid,'%f',1);
      MRDS.freq = 1/dt;
    end

    if (strcmp(string,'$NVAR')),
      MRDS.cols=fscanf(fid,'%d',1);
    end

    if (strcmp(string,'$VAR')),
      MRDS.vars(end+1).name = fscanf(fid,'%s',1);
      MRDS.vars(end).unit   = '-';
    end

    if (strcmp(string,'$OUT')),
      MRDS.rows=fscanf(fid,'%d',1);
    end

    if (strcmp(string,'$END_HEADER')),
      break;
    end

  end

  % read the data
  fscanf(fid,'%c',1);

else                                % this is a CLMCPLOT file

  specs=fscanf(fid,'%d %d %d %f',4);  % [dummy,cols,rows,freq]
  MRDS.cols = specs(2);
  MRDS.rows = specs(3);
  MRDS.freq = specs(4);
  MRDS.vars = [];

  % read all variable names
  for i=1:MRDS.cols,
    MRDS.vars(i).name=fscanf(fid,'%s',1);
    MRDS.vars(i).unit=fscanf(fid,'%s',1);
  end;
  fscanf(fid,'%c',3); % there are three characters which must be skipped

end

% read the data
MRDS.data = fread(fid, [MRDS.cols,inf],'float32');
[MRDS.cols,MRDS.rows] = size(MRDS.data);
MRDS.data = MRDS.data';
fclose(fid);

MRDS.t     = (1:MRDS.rows)'/MRDS.freq;  % the time column
MRDS.fname = fname;

% build the string for showing the variable names

temp = sprintf('%s [%s]',MRDS.vars(1).name,MRDS.vars(1).unit);
string=sprintf('%25s= %% .4g',temp);
MRDS.varnames{1}=MRDS.vars(1).name;
for i=2:MRDS.cols,
	temp = sprintf('%s [%s]',MRDS.vars(i).name,MRDS.vars(i).unit);
	string=[string,'|',sprintf('%25s= %% .4g',temp)];
  MRDS.varnames{i}=MRDS.vars(i).name;
end;

MRDS.string=string;
set(MRDS.lptr,'Value',1,'String',sprintf(MRDS.string,MRDS.data(1,:)));
MRDS.cursor = 1;
set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),MRDS.cursor));

% fix all plots

clmcplot_update_subplots;

% write the file name

set(MRDS.nptr,'String',sprintf('Variables of %s',fname_store));

%------------------------------------------------------------------------------------
function clmcplot_data()
% creates the window with the data plots

global MRDS;

% get the screen size
scrollbar_width = 25;
if (size(MRDS.screensize,1) > 1),
  fig_size = MRDS.screensize(1,:);
else
  s = MRDS.screensize;
  fig_size(1) = s(1)+1;
  fig_size(2) = s(2)+150;
  fig_size(3) = s(3)-260-(fig_size(1)-s(1))-scrollbar_width;
  fig_size(4) = s(4)-50-(fig_size(2)-s(2));
end

if ishandle(451),
  delete(451);
end;

ptr = figure(451);   % the window number is arbitrarily set to 451
set(ptr,'Position',fig_size);
MRDS.dptr = ptr;

set(ptr,...
'Units','Pixels', ...
'Color',[0.8 0.8 0.8], ...
'Name','CLMCPLOT-Data', ...
'KeyPressFcn','clmcplot_functions(''key_press'');',...
'WindowButtonDownFcn','clmcplot_functions(''cursor'');',...
'NumberTitle','off', ...
'Position',fig_size, ...
'MenuBar','none',...	
'Tag','');

clmcplot_create_subplots(1);

%------------------------------------------------------------------------------------------
function clmcplot_create_subplots(flag)
% create axes in data window

global MRDS;

MRDS.axes=[];
MRDS.t=(1:MRDS.rows)'/MRDS.freq;
if flag,
	MRDS.cursor=1;
  write_datapoint;
end
set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),MRDS.cursor));
MRDS.axes_cursors=[];
figure(MRDS.dptr);
clf;

for i=1:MRDS.n_subplots,
	
	% the main windows
	rect = [.05 0.05+(i-1)*0.95/MRDS.n_subplots 0.7 0.95/MRDS.n_subplots*0.8];
	MRDS.axes(i,1)=axes('position',rect,'XGrid','on','YGrid','on','FontSize',MRDS.fontsize);
	set(MRDS.axes(i,1),'Box','on');
	axis([0 MRDS.rows/MRDS.freq 0 1]);
	MRDS.axes_cursors(i,1) = line(MRDS.t(MRDS.cursor)*[1;1],[0;1]);
	set(MRDS.axes_cursors(i,1),'Color',[.8 .0 .0]);
	
	% the zoom window
	rect = [.8 0.05+(i-1)*0.95/MRDS.n_subplots 0.18 0.95/MRDS.n_subplots*0.8];
	MRDS.axes(i,2)=axes('position',rect,'XGrid','on','YGrid','on','FontSize',MRDS.fontsize);
	set(MRDS.axes(i,2),'Box','on');
	axis([-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom 0 1]);
	MRDS.axes_cursors(i,2) = line(MRDS.t(MRDS.cursor)*[1;1],[0;1]);
	set(MRDS.axes_cursors(i,2),'Color',[.8 .0 .0]);
	if flag,
		axes_data(i).values=[];
    axes_data(i).names={};
	else
		axes_data(i).values=MRDS.axes_data(i).values;		
    axes_data(i).names=MRDS.axes_data(i).names;
	end;
end;

MRDS.axes_data = axes_data;

%------------------------------------------------------------------------------------------
function clmcplot_update_subplots
% create axes in data window

global MRDS;

% check whether all values are properly refered to

for i=1:MRDS.n_subplots,
  names = MRDS.axes_data(i).names;
  MRDS.axes_data(i).values=[];
  MRDS.axes_data(i).names={};
  for j=1:length(names),
    ind=find(strcmp(names{j},MRDS.varnames));
    if ~isempty(ind),
      MRDS.axes_data(i).values=[MRDS.axes_data(i).values ind];
      MRDS.axes_data(i).names{end+1}=MRDS.varnames{ind};
    end
  end
end;

% update the plots

MRDS.t=(1:MRDS.rows)'/MRDS.freq;

if MRDS.cursor > MRDS.rows || MRDS.cursor < 1,
	MRDS.cursor = 1;
  write_datapoint;
	set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),...
	MRDS.cursor));
end;

for i=1:MRDS.n_subplots,
	if isempty(MRDS.axes_data(i).values),
		clear_subplot(i);
	else
		redraw_subplot(i);
	end	
end;

%------------------------------------------------------------------------------------------
function clmcplot_zoom
% adjust the axes of zoom windows

global MRDS;

for i=1:MRDS.n_subplots,
	% the zoom window
	a=MRDS.axes(i,2);
	set(a,'XLim',[-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom]+MRDS.t(MRDS.cursor));
	t=get(a,'YLim');
	set(MRDS.axes_cursors(i,2),'YData',t);
end;

%------------------------------------------------------------------------------------------
function clmcplot_varlist
% processes the callback for the variable list (added to accommodate phase plots)

global MRDS;

if (MRDS.flag == 7), %---add/sub 1--
	if (toc - MRDS.time <= 6),
		MRDS.var1 = get(MRDS.lptr, 'value');
		MRDS.flag = 8;
  else
		MRDS.time = toc;
		MRDS.flag = 1;
	end;
elseif (MRDS.flag == 8), %---add/sub 2---
	MRDS.var2 = get(MRDS.lptr, 'value');
	MRDS.flag = 0;
	MRDS.time = toc;
	answer=questdlg('Add or Subtract?','Input','+','-','-');
	if answer=='+',
		MRDS.data = [MRDS.data, MRDS.data(:,MRDS.var1)+MRDS.data(:,MRDS.var2)];
		dname = sprintf('%s+%s',MRDS.vars(MRDS.var1).name,MRDS.vars(MRDS.var2).name);
	else
		MRDS.data = [MRDS.data, MRDS.data(:,MRDS.var1)-MRDS.data(:,MRDS.var2)];
		dname = sprintf('%s-%s',MRDS.vars(MRDS.var1).name,MRDS.vars(MRDS.var2).name);
	end	
	MRDS.cols = MRDS.cols + 1;
	dunit = '-';
	dvar.name = dname;
	dvar.unit = dunit;
	MRDS.vars = [MRDS.vars, dvar];
	temp = sprintf('%s [%s]', dname, dunit);
	MRDS.string = [MRDS.string,'|',sprintf('%20s= %% .4g',temp)];
	set(MRDS.lptr,'String',sprintf(MRDS.string,MRDS.data(1,:)));
	MRDS.flag = 0;
	MRDS.time = toc;
elseif (MRDS.flag == 4), %---Phase Plot 1---
	if (toc - MRDS.time <= 6),
		MRDS.var1 = get(MRDS.lptr, 'value');
		MRDS.flag = 5;
  else
		MRDS.time = toc;
		MRDS.flag = 1;
	end;
elseif (MRDS.flag == 5), %---Phase Plot 2---
	MRDS.var2 = get(MRDS.lptr, 'value');
	MRDS.flag = 0;
	MRDS.time = toc;
	figure;
	v1=MRDS.var1; 
	v2=MRDS.var2;
	plot(MRDS.data(:,v1),MRDS.data(:,v2));
	xl = sprintf('%s (%s)', MRDS.vars(v1).name, MRDS.vars(v1).unit);
	yl = sprintf('%s (%s)', MRDS.vars(v2).name, MRDS.vars(v2).unit);
	a=xlabel(xl,'FontSize',MRDS.fontsize);
	set(a,'Interpreter','none','FontSize',MRDS.fontsize);
	a=ylabel(yl,'FontSize',MRDS.fontsize);
	set(a,'Interpreter','none','FontSize',MRDS.fontsize);
elseif (MRDS.flag == 6), %---Differentiation---
	if (toc - MRDS.time <= 6),
		prompt = {'Filter order:','Filter cutoff frequency:'};
		def = {'2','0.2'};
		reply = inputdlg(prompt, 'Differentiation Filter Options', 1, def);
		order = sscanf(char(reply(1)),'%f');
		cutoff = sscanf(char(reply(2)), '%f');
		[b,a] = butter(order, cutoff);
		dvar = get(MRDS.lptr, 'value');
		x = diff(MRDS.data(:,dvar));
		x = [x(1); x];
		x = x * MRDS.freq;
		if order ~= 0 && cutoff ~= 1,
			filt_x = filtfilt(b, a, x);
		else 
			filt_x = x;
		end;
		MRDS.data = [MRDS.data, filt_x];
		MRDS.cols = MRDS.cols + 1;
		dname = sprintf('d%s/dt', MRDS.vars(dvar).name);
		dunit = sprintf('%s/s', MRDS.vars(dvar).unit);
		dvar = [];
		dvar.name = dname;
		dvar.unit = dunit;
		MRDS.vars = [MRDS.vars, dvar];
		MRDS.varnames{end+1} = dname;
		temp = sprintf('%s [%s]', dname, dunit);
		MRDS.string = [MRDS.string,'|',sprintf('%20s= %% .4g',temp)];
		set(MRDS.lptr,'String',sprintf(MRDS.string,MRDS.data(1,:)));
		MRDS.flag = 0;
		MRDS.time = toc;
  else
		MRDS.time = toc;
		MRDS.flag = 1;
	end;
else %---Normal action---
	MRDS.time=toc;
	MRDS.flag=1;
end;

%------------------------------------------------------------------------------------------
function clmcplot_cursor
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% was the click over a subplot?

a=overobj('axes');

if ~isempty(a) && isempty(find(MRDS.axes==a, 1)),
	return;
end;
if ~isempty(a) && toc-MRDS.time < 5,  % if yes and only 5 secs ago the user clicked on variable,
	% the variable is added to plots
	MRDS.time = toc-6;                                                % this makes sure that a subsequent click does not cause
	% any action anymore.
	
	% which subplots?
	[i,j]=find(MRDS.axes==a);
	
	if MRDS.flag == 2 || (MRDS.flag == 3 && length(MRDS.axes_data(i).values) < 2),
		clear_subplot(i);
	elseif MRDS.flag==1 || MRDS.flag==3,  % add or delete variable
		if MRDS.flag == 1,
			% which variable is highlighted?
			v=get(MRDS.lptr,'Value');
			% add variable to the plot data
			ind=[];
			if ~isempty(MRDS.axes_data(i).values),
				ind = find(v==MRDS.axes_data(i).values);
			end;
			if isempty(ind),
				MRDS.axes_data(i).values = [MRDS.axes_data(i).values v];
				MRDS.axes_data(i).names{end+1}  = MRDS.varnames{v};
				v = MRDS.axes_data(i).values;
      else
				return;
			end;
		elseif MRDS.flag == 3,
			MRDS.axes_data(i).values(end)=[];
			MRDS.axes_data(i).names=MRDS.axes_data(i).names(1:end-1);
			v=MRDS.axes_data(i).values;
		end;
		redraw_subplot(i);
	end;
elseif ~isempty(a),
	% move the cursor
	MRDS.flag = 0;
	do_cursor;
end;

%------------------------------------------------------------------------------------------
function clear_subplot(i)
% clears a subplot

global MRDS;

MRDS.axes_data(i).values=[];
MRDS.axes_data(i).names={};
axes(MRDS.axes(i,1));
cla;
legend('off');
MRDS.axes_cursors(i,1) = line([1;1]*MRDS.t(MRDS.cursor),[0;1]);
set(MRDS.axes_cursors(i,1),'Color',[.8 .0 .0]);
axes(MRDS.axes(i,2));
cla;
MRDS.axes_cursors(i,2) = line([1;1]*MRDS.t(MRDS.cursor),[0;1]);
set(MRDS.axes_cursors(i,2),'Color',[.8 .0 .0]);
redraw_subplot(i);

%------------------------------------------------------------------------------------------
function redraw_subplot(i)
% redraws an entire subplot

global MRDS;

% plot in full plot
v = MRDS.axes_data(i).values;

if ~isempty(v),
	axes(MRDS.axes(i,1));
	plot(MRDS.t,MRDS.data(:,v));
	set(MRDS.axes(i,1),'FontSize',MRDS.fontsize);
	grid on;
	
	% since the plot command erased the cursor, recreate it
	a=axis;
	MRDS.axes_cursors(i,1) = line([1;1]*MRDS.t(MRDS.cursor),[a(3);a(4)]);
	set(MRDS.axes_cursors(i,1),'Color',[.8 .0 .0]);
	set(MRDS.axes(i,1),'XLim',[0 MRDS.rows/MRDS.freq],'YLim',[a(3) a(4)]);
	if ~strcmp(MRDS.legend_pos,'off')
	  legend(MRDS.axes_data(i).names,'Location',MRDS.legend_pos, ...
		 'Interpreter','none');
	end
else
	set(MRDS.axes(i,1),'XLim',[0 MRDS.rows/MRDS.freq],'YLim',[0 1]);
	set(MRDS.axes_cursors(i,1),'XData',MRDS.t(MRDS.cursor)*[1;1],'YData',[0 1]);
end;

% plot in the zoom window
if ~isempty(v),
	axes(MRDS.axes(i,2));
	plot(MRDS.t,MRDS.data(:,v));
	set(MRDS.axes(i,2),'FontSize',MRDS.fontsize);
	grid on;
	set(MRDS.axes(i,2),'XLim',...
	[-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom]+MRDS.t(MRDS.cursor));
	a=axis;
	MRDS.axes_cursors(i,2) = line([1;1]*MRDS.t(MRDS.cursor),[a(3);a(4)]);
	a=axis;
	set(MRDS.axes_cursors(i,2),'Color',[.8 .0 .0],'YData',[a(3); a(4)]);
else
	set(MRDS.axes(i,2),'XLim',[-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom],'YLim',[0 1]);
	set(MRDS.axes_cursors(i,2),'XData',MRDS.t(MRDS.cursor)*[1;1],'YData',[0 1]);
end;

%------------------------------------------------------------------------------------------
function do_cursor
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% move the cursor
a=overobj('axes');
p=get(a,'CurrentPoint');
n = round(p(1,1)*MRDS.freq);
if n > MRDS.rows,
	n = MRDS.rows;
elseif n < 1,
	n = 1;
end

if n == MRDS.cursor,
	return;
end;
MRDS.cursor = n;
write_datapoint;
update_after_cursor_move;

%------------------------------------------------------------------------------------------
function clear_all
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% move the cursor
for j=1:MRDS.n_subplots,
	delete(MRDS.axes(j,1));
	delete(MRDS.axes(j,2));
	MRDS.axes_data(j).values=[];
	MRDS.axes_data(j).names={};
end;
clmcplot_create_subplots(1);

%------------------------------------------------------------------------------------------
function key_press
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% which key was pressed? f=forward b=backward

c=get(MRDS.dptr,'CurrentCharacter');

if strcmp(c,'f') && MRDS.cursor < MRDS.rows,
	MRDS.cursor = MRDS.cursor + 1;
  write_datapoint;
elseif strcmp(c,'b') && MRDS.cursor > 1,
	MRDS.cursor = MRDS.cursor - 1;
  write_datapoint;
else
	return;
end;

update_after_cursor_move;

%------------------------------------------------------------------------------------------
function update_after_cursor_move
% performs necessary window updates after the cursor was moved

global MRDS;

set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),MRDS.cursor));

for j=1:MRDS.n_subplots,
	set(MRDS.axes_cursors(j,1),'XData',[1;1]*MRDS.t(MRDS.cursor));
	set(MRDS.axes_cursors(j,2),'XData',[-100;-100]);
	set(MRDS.axes(j,2),'XLim',[MRDS.t(MRDS.cursor)-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.t(MRDS.cursor)+MRDS.rows/MRDS.freq/MRDS.zoom]);
	% note that the repetition of the command below is necessary for proper display
	t=get(MRDS.axes(j,2),'YLim');
	set(MRDS.axes_cursors(j,2),'XData',[1;1]*MRDS.t(MRDS.cursor),'YData',t);
	t=get(MRDS.axes(j,2),'YLim');
	set(MRDS.axes_cursors(j,2),'XData',[1;1]*MRDS.t(MRDS.cursor),'YData',t);
end;

if ~isempty(MRDS.string),
	set(MRDS.lptr,'String',sprintf(MRDS.string,MRDS.data(MRDS.cursor,:)));
end;


%------------------------------------------------------------------------------------------
function add_subplot(num)
% adds a subplot

global MRDS;

for i=1:num,
  MRDS.n_subplots = MRDS.n_subplots+1;
  MRDS.axes_data=[MRDS.axes_data MRDS.axes_data(1)];
  MRDS.axes_data(MRDS.n_subplots).values=[];
  MRDS.axes_data(MRDS.n_subplots).names={};
end
clmcplot_create_subplots(0);
for i=1:MRDS.n_subplots,
	redraw_subplot(i);
end;
update_after_cursor_move;

%------------------------------------------------------------------------------------------
function delete_subplot(num)
% deletes a subplot

global MRDS;

for i=1:num,
  if MRDS.n_subplots > 1,
    MRDS.axes_data(MRDS.n_subplots)=[];
    MRDS.n_subplots = MRDS.n_subplots-1;
  end
end
clmcplot_create_subplots(0);
for i=1:MRDS.n_subplots,
  redraw_subplot(i);
end


%------------------------------------------------------------------------------------------
function save_view
% saves the current view

global MRDS;

prompt = {'View Name:'};
def = {sprintf('View_%d',length(MRDS.views)+1-2)};
reply = inputdlg(prompt, 'Input', 1, def);
if isempty(reply)
  return;
end

% extract the current view
temp.name = reply{1};
temp.n_subplots = MRDS.n_subplots;
for i=1:MRDS.n_subplots,
  temp.axes_data(i)=MRDS.axes_data(i);
end

if isempty(MRDS.views)
  MRDS.views = temp;
else
  MRDS.views(end+1) = temp;
end

rebuild_view_list(1);

views = MRDS.views;
save('.clmcplot','views');

%------------------------------------------------------------------------------------------
function rebuild_view_list(value)
% rebuilds the list of views

global MRDS;

list={};

for i=1:length(MRDS.views),
	list{i} = MRDS.views(i).name;
end

set(MRDS.vptr,'Value',value,'String',list);


%------------------------------------------------------------------------------------------
function clmcplot_viewlist
% manages the view list

global MRDS;

if (MRDS.flag == 100), % delete a view
	if (toc - MRDS.time <= 6),
		listitem = get(MRDS.vptr, 'value');
    if listitem <= 2,
      disp(sprintf('The "current" and "last" view cannot be deleted'));
      return;
    end
    MRDS.views(listitem)=[];
    rebuild_view_list(length(MRDS.views));
    views = MRDS.views;
    save('.clmcplot','views');
    set(MRDS.vptr,'Value',1);
    MRDS.flag == 0;
    return;
	end;
end

listitem = get(MRDS.vptr, 'value');

if listitem == 1,
  return;
end

% save old view in last
if (listitem > 2)
  MRDS.views(2).n_subplots = MRDS.n_subplots;
  MRDS.views(2).axes_data  = MRDS.axes_data;
end

% apply a view
num = MRDS.views(listitem).n_subplots - MRDS.n_subplots;
if num > 0
  add_subplot(num),
elseif num < 0
  delete_subplot(-num);
end

MRDS.axes_data = MRDS.views(listitem).axes_data;

clmcplot_update_subplots;

set(MRDS.vptr,'Value',1);

%------------------------------------------------------------------------------------------
function views=default_views
% initializes the default views

global MRDS;

% extract the current view
temp.name = 'current';
temp.n_subplots = MRDS.n_subplots;
for i=1:MRDS.n_subplots,
  temp.axes_data(i).values=[];
  temp.axes_data(i).names={};
end

temp(2) = temp;
temp(2).name='last';

views=temp;

%------------------------------------------------------------------------------------------
function clmcplot_toggle_legend

global MRDS;

if strcmp(MRDS.legend_pos,'NorthEast'),
  MRDS.legend_pos = 'NorthWest';
elseif strcmp(MRDS.legend_pos,'NorthWest'),
  MRDS.legend_pos = 'off';
else
  MRDS.legend_pos = 'NorthEast';
end

for i=1:MRDS.n_subplots,
  redraw_subplot(i);
end

  
