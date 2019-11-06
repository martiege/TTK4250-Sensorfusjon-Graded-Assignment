%%%%%%%%%%%%%%%%%

function prcdone(iloop,itot,title,dprc,tstart)
%PRCDONE calculate the percentage of loop done in a running loop and
% display an estimate of remaining run time
% PRCDONE(ILOOP,ITOT,DPRC) convert iloop into iloop/itot*100% and display
% the result by dprc incrementations
% ILOOP is the current loop id (assumes iloop = 1:itot)
% ITOT is the total number of iterations
% TITLE (string, optional) is the title of the loop for display
% If using dprc set title to '' if you don't want a title
% DPRC (optional) is the incrementations to display (in %).
% Default is 10%
% TSTART is the stopwatch time just before the loop is entered. Use e.g. "tstart
% = tic"; just above your loop. This function uses toc to determine
% the elapsed time.
% Warning: this script will add time on your loop
%
% Example:
%
% time_of_start=tic;
% for i0 = 1:100
% pause(0.15)
% prcdone(i0,100,'test_loop',10,time_of_start)
% end

% Original Author: Arnaud Laurent
% Creation : Dec 4th 2012
% Updates: Matthew Gruber
% Nov 1st 2017
% MATLAB version: R2015a
%
% Last modified: November 11th 2017

if nargin<3
title = ' unnamed ';
end

if nargin<4
dprc=10;
end

frac_now = dprc*(ceil(iloop/itot*100/dprc));
frac_next = dprc*(ceil((iloop+1)/itot*100/dprc));

if iloop == 1
disp(['Starting ' title ' loop'])
elseif iloop==itot
disp(['Done ' title ' loop'])
elseif frac_next>frac_now

if nargin<5
disp([num2str(frac_now) '% completed of "' title '" loop.'])
disp('')
else
rate=iloop/toc(tstart); % iterations per second
i_to_do=itot-iloop; % iterations

timeleft= i_to_do/rate; % seconds
hours=(timeleft-mod(timeleft,3600))/3600;
timeleft=timeleft-hours*3600;
minutes=(timeleft-mod(timeleft,60))/60;
timeleft=timeleft-minutes*60;
disp([num2str(frac_now) '% completed of "' title '" loop. ' num2str(hours) ' hours, ' num2str(minutes) ' minutes and ' num2str(timeleft) ' seconds remaining at ' datestr(clock) '.'])
end
end

%%%%%%%%%%%%%%%%% 