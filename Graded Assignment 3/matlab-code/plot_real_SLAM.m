style = hgexport('factorystyle');
style.bounds = 'tight';
style.Format = 'eps';
style.Width = 6;
style.Height = 2.5;
style.Resolution = 300;
style.Units = 'inch';
style.FixedFontSize = 12;

set(0,'DefaultAxesColorOrder',cbrewer('qual','Set2',3))
set(0,'DefaultLineLineWidth',1.2)  

%% 
f = figure(2); clf;  hold on; grid on; axis equal;
scatter(eta(4:2:end), eta(5:2:end), 'rx');
s = scatter(Lo_m(timeGps < timeOdo(N)), La_m(timeGps < timeOdo(N)), 'filled');
plot(xupd(1, 2:(mk-1)), xupd(2, 2:(mk-1)));
s.MarkerFaceAlpha = 0.4;

title('Estimated trajectory and landmarks vs. GPS')
if export_plots
    hgexport(f,'figures/ga_3_real_trajectory.eps',style,'Format','eps');
end

% what can we do with consistency..? divide by the number of associated
% measurements? 
alpha = 0.05;
ANIS = mean(NIS)
ACI = chi2inv([alpha/2; 1 - alpha/2], 1)/N % NOT CORRECT NOW
CI = chi2inv([alpha/2; 1 - alpha/2], 1); % NOT CORRECT NOW
warning('These consistency intervals have wrong degrees of freedom')

f = figure(3); clf;
plot(NIS);
hold on;
plot(1:N, NIS(1:N));
insideCI = mean((CI(1) < NIS) .* (NIS <= CI(2)))*100;
plot([1, N], (CI*ones(1, 2))','r--');
title(sprintf('NIS over time, with %0.1f%% inside %0.1f%% CI', insideCI, (1-alpha)*100));
grid on;
ylabel('NIS');
xlabel('time');
if export_plots
    hgexport(f,'figures/ga_3_real_NIS.eps',style,'Format','eps');
end

f = figure(4); clf;
hold on;
plot(NEESpos); grid on; hold on;
ylabel('NEES in position');
insideCI = mean((CI(1) < NEESpos) .* (NEESpos <= CI(2)))*100;  % Probably not correct
plot([1, N], (CI*ones(1, 2))','r--');
title(sprintf('NEES over time, with %0.1f%% inside %0.1f%% CI', insideCI, (1-alpha)*100));
if export_plots
    hgexport(f,'figures/ga_3_real_NEES.eps',style,'Format','eps');
end
