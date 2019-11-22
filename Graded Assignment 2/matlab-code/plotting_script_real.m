% add style:
style = hgexport('factorystyle');
style.bounds = 'tight';
style.Format = 'eps';
style.Width = 8;
style.Height = 6;
style.Resolution = 300;
style.Units = 'inch';
style.FixedFontSize = 10;

f = figure(1);
clf;
plot3(xest(2, K_start:N), xest(1, K_start:N), -xest(3, K_start:N));
hold on;
plot3(zGNSS(2, K_start:GNSSk), zGNSS(1, K_start:GNSSk), -zGNSS(3, K_start:GNSSk))
grid on; axis equal
xlabel('East [m]')
ylabel('North [m]')
%zlabel('Altitude [m]')
title('Position estimate vs. GNSS in yx-plane');
legend('Position estimate', 'GNSS measurements', 'Location', 'southwest');
view(2)
if export_plots
    hgexport(f,'figures/ga_2_real_trajectory.eps',style,'Format','eps');
end

%%
eul = quat2eul(xest(7:10, :))*180/pi;
f = figure(2); clf; hold on;
subplot(5,1,1);
plot(timeIMU(K_start:N) - timeIMU(1), xest(1:3, K_start:N))
xlim([0, 3670]);
grid on;
title('NED position [m]')
subplot(5,1,2);
plot(timeIMU(K_start:N) - timeIMU(1), xest(4:6, K_start:N))
xlim([0, 3670]);
grid on;
title('Velocitites [m/s]')
subplot(5,1,3);
plot(timeIMU(K_start:N) - timeIMU(1), eul(:, K_start:N))
xlim([0, 3670]);
grid on;
title('euler angles [deg]')
subplot(5, 1, 4)
plot(timeIMU(K_start:N) - timeIMU(1), xest(11:13, K_start:N))
xlim([0, 3670]);
grid on;
title('Accl bias [m/s^2]')
subplot(5, 1, 5)
plot(timeIMU(K_start:N) - timeIMU(1), xest(14:16, K_start:N)*180/pi * 3600)
xlim([0, 3670]);
grid on;
title('Gyro bias [rad/s]')
if export_plots
    hgexport(f, 'figures/ga_2_real_state.eps', style, 'Format', 'eps');
end

style.Height = 3;

f = figure(3);
alpha = 0.05;
CI3 = chi2inv([alpha/2; 1 - alpha/2; 0.5], 3);
clf;
semilogy(timeGNSS(1:(GNSSk - 1)) - timeIMU(1), (NIS(1:GNSSk - 1)));
grid on;
hold on;
semilogy([0, timeIMU(N) - timeIMU(1)], (CI3*ones(1,2))', 'r--');
xlim([0, 3670]);
insideCI = mean((CI3(1) <= NIS).* (NIS <= CI3(2)));
title(sprintf('NIS in log scale (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));
if export_plots
    hgexport(f, 'figures/ga_2_real_consistency.eps', style, 'Format', 'eps');
end

ANIS = mean(NIS);

figure(4); clf;
gaussCompare = sum(randn(3, numel(NIS)).^2, 1);
boxplot([NIS', gaussCompare'],'notch','on',...
        'labels',{'NIS','gauss'});
grid on;

f = figure(5); clf;
plot(timeIMU(K_start:N) - timeIMU(1), eul(:, K_start:N))
%xlim([0, 3670]);
grid on;
title('Euler angles [deg]')
if export_plots
    hgexport(f, 'figures/ga_2_real_bad_heading.eps', style, 'Format', 'eps');
end