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
set(0,'DefaultFigureColormap','default');     

%% 
R = rotmat2d(-30*pi/180);
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
warning('These consistency intervals have wrong degrees of freedom')

mk = mk - 1;

f = figure(3); clf;
hold on;
plot(3:mk, NIS(3:mk));
insideCI = mean((CI(1, :) < NIS) .* (NIS <= CI(2, :)))*100;
plot(3:mk, CI(1, 3:mk), 'r--');
plot(3:mk, CI(2, 3:mk), 'r--');
title(sprintf('NIS over time, with %0.1f%% inside %0.1f%% CI', insideCI, (1-alpha)*100));
grid on;
ylabel('NIS');
xlabel('time');
if export_plots
    hgexport(f,'figures/ga_3_real_NIS.eps',style,'Format','eps');
end

f = figure(10); clf;
subplot(1,2,1);
imagesc(Pupd{mk});
%axis equal;
title('Final covariance matrix');

subplot(1,2,2);
imagesc(inv(Pupd{mk}));
%axis equal;
title('Final information matrix');
colormap('default')

%colormap(cbrewer('seq','YlOrRd',128)); 

if export_plots
    hgexport(f,'figures/ga_3_real_P.eps',style,'Format','eps');
end


% RMSEpos = sqrt(sum((xupd(1:2, 2:mk) - [Lo_m(2:mk)'; La_m(2:mk)']).^2, 1));
% figure(8); clf;
% plot(RMSEpos); grid on;
% title('Position RMSE');
