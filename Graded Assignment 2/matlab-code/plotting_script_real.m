% add style:
style = hgexport('factorystyle');
style.bounds = 'tight';
style.Format = 'eps';
style.Width = 8;
style.Height = 6;
style.Resolution = 300;
style.Units = 'inch';
style.FixedFontSize = 10;

figure(1);
clf;
plot3(xest(2, K_start:N), xest(1, K_start:N), -xest(3, K_start:N));
hold on;
plot3(zGNSS(2, K_start:GNSSk), zGNSS(1, K_start:GNSSk), -zGNSS(3, K_start:GNSSk))
grid on; axis equal
xlabel('East [m]')
ylabel('North [m]')
zlabel('Altitude [m]')
if export_plots
    hgexport(f,'figures/ga_2_real_trajectory.eps',style,'Format','eps');
end

%%
eul = quat2eul(xest(7:10, :))*180/pi;
figure(2); clf; hold on;
subplot(5,1,1);
plot(timeIMU(K_start:N) - timeIMU(1), xest(1:3, K_start:N))
grid on;
ylabel('NED position [m]')
subplot(5,1,2);
plot(timeIMU(K_start:N) - timeIMU(1), xest(4:6, K_start:N))
grid on;
ylabel('Velocitites [m/s]')
subplot(5,1,3);
plot(timeIMU(K_start:N) - timeIMU(1), eul(:, K_start:N))
grid on;
ylabel('euler angles [deg]')
legend('\phi', '\theta', '\psi')
subplot(5, 1, 4)
plot(timeIMU(K_start:N) - timeIMU(1), xest(11:13, K_start:N))
grid on;
ylabel('Accl bias [m/s^2]')
subplot(5, 1, 5)
plot(timeIMU(K_start:N) - timeIMU(1), xest(14:16, K_start:N)*180/pi * 3600)
grid on;
ylabel('Gyro bias [rad/s]')
if export_plots
    hgexport(f, 'figures/ga_2_real_state.eps', style, 'Format', 'eps');
end

figure(3);
alpha = 0.05;
CI3 = chi2inv([alpha/2; 1 - alpha/2; 0.5], 3);
clf;
plot(timeGNSS(1:(GNSSk - 1)) - timeIMU(1), NIS(1:GNSSk - 1));
grid on;
hold on;
plot([0, timeIMU(N) - timeIMU(1)], (CI3*ones(1,2))', 'r--');
insideCI = mean((CI3(1) <= NIS).* (NIS <= CI3(2)));
title(sprintf('NIS (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));
if export_plots
    hgexport(f, 'figures/ga_2_real_errors.eps', style, 'Format', 'eps');
end

figure(4); clf;
gaussCompare = sum(randn(3, numel(NIS)).^2, 1);
boxplot([NIS', gaussCompare'],'notch','on',...
        'labels',{'NIS','gauss'});
grid on;