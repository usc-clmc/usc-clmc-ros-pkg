% CLMCPLOT is a matlab tool to display data trace in space and time.
% Just type 'clmcplot" to initiate the program, and use the buttons
% in the indicated, straightforward way.

% Copyright 2008 Stefan Schaal
% Computational Learning and Motor Control Laboratory
% University of Southern California
% December 1997-2008
function clmcplot(screensize)

global MRDS;

% is CLMCPLOT already running? Check whether the windows exist.
if ~isempty(MRDS) && ishandle(MRDS.cptr) && ishandle(MRDS.dptr),
	disp('CLMCPLOT is already running!');
	return;
end

MRDS=[];

if ~exist('screensize') || isempty(screensize)
  store_units = get(0,'Units');
  set(0,'Units','pixels');
  screensize = get(0,'ScreenSize');
  set(0,'Units',store_units);
end

MRDS.screensize = screensize;

tic;
clmcplot_functions('init');

