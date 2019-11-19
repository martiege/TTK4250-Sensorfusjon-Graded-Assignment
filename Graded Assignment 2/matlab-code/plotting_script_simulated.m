% add style:
style = hgexport('factorystyle');
style.bounds = 'tight';
style.Format = 'eps';
style.Width = 8;
style.Height = 6;
style.Resolution = 300;
style.Units = 'inch';
style.FixedFontSize = 10;

if plot_path
    f = figure(1);
    clf;
    plot3(xest(2, 1:N), xest(1, 1:N), -xest(3, 1:N));
    hold on;
    plot3(zGNSS(2, 1:GNSSk), zGNSS(1, 1:GNSSk), -zGNSS(3, 1:GNSSk))
    grid on; 
    xlabel('East [m]')
    ylabel('North [m]')
    %zlabel('Altitude [m]')
    view(2)
    title('Position estimate vs. GNSS in yx-plane');
    legend('Position estimate', 'GNSS measurements', 'Location', 'southwest');
    if export_plots
        hgexport(f,'figures/ga_2_sim_trajectory.eps',style,'Format','eps');
    end
end
% state estimate plot
eul = quat2eul(xest(7:10, :));
eul_true = quat2eul(xtrue(7:10, :));

if plot_state_estimates
    f = figure(2); clf; hold on;

    subplot(5,1,1);
    plot((0:(N-1))*dt, xest(1:3, 1:N))
    grid on;
    title('NED position [m]')
    legend('North', 'East', 'Down')

    subplot(5,1,2);
    plot((0:(N-1))*dt, xest(4:6, 1:N))
    grid on;
    title('Velocitites [m/s]')
    legend('North', 'East', 'Down')

    subplot(5,1,3);
    plot((0:(N-1))*dt, eul(:, 1:N)*180/pi)
    hold on
    %plot((0:(N-1))*dt, euler_out(:, 1:N)*180/pi)
    grid on;
    title('Euler angles [deg]')
    legend('\phi', '\theta', '\psi')

    subplot(5, 1, 4)
    plot((0:(N-1))*dt, xest(11:13, 1:N))
    grid on;
    title('Acceleration bias [m/s^2]')
    legend('x', 'y', 'z')
    
    subplot(5, 1, 5)
    plot((0:(N-1))*dt, xest(14:16, 1:N)*180/pi * 3600)
    grid on;
    title('Gyro bias [deg/h]')
    legend('x', 'y', 'z')

    %suptitle('State estimates');
    if export_plots
        hgexport(f, 'figures/ga_2_sim_state.eps', style, 'Format', 'eps');
    end
end

% state error plots
if plot_state_error
    f = figure(3); clf; hold on;

    subplot(5,1,1);
    plot((0:(N-1))*dt, deltaX(1:3,1:N))
    grid on;
    title('NED position error [m]')
    legend(sprintf('North (%.3g)', sqrt(mean(deltaX(1, 1:N).^2))),...
        sprintf('East (%.3g)', sqrt(mean(deltaX(2, 1:N).^2))),...
        sprintf('Down (%.3g)', sqrt(mean(deltaX(3, 1:N).^2))))

    subplot(5,1,2);
    plot((0:(N-1))*dt, deltaX(4:6, 1:N))
    grid on;
    title('Velocity error [m/s]')
    legend(sprintf('North (%.3g)', sqrt(mean(deltaX(4, 1:N).^2))),...
        sprintf('East (%.3g)', sqrt(mean(deltaX(5, 1:N).^2))),...
        sprintf('Down (%.3g)', sqrt(mean(deltaX(6, 1:N).^2))))

    subplot(5,1,3);
    plot((0:(N-1))*dt, wrapToPi(eul(:, 1:N) - eul_true(:, 1:N))*180/pi)
    grid on;
    title('Euler angle error [deg]')
    legend(sprintf('\\phi (%.3g)', sqrt(mean((eul(1, 1:N) - eul_true(1, 1:N)).^2))),...
        sprintf('\\theta (%.3g)', sqrt(mean((eul(2, 1:N) - eul_true(2, 1:N)).^2))),...
        sprintf('\\psi (%.3g)', sqrt(mean((eul(3, 1:N) - eul_true(3, 1:N)).^2))))

    subplot(5, 1, 4)
    plot((0:(N-1))*dt, deltaX(10:12, 1:N))
    grid on;
    title('Accerelation bias error [m/s^2]')
    legend(sprintf('x (%.3g)', sqrt(mean(deltaX(1, 1:N).^2))),...
        sprintf('y (%.3g)', sqrt(mean(deltaX(11, 1:N).^2))),...
        sprintf('z (%.3g)', sqrt(mean(deltaX(12, 1:N).^2))))

    subplot(5, 1, 5)
    plot((0:(N-1))*dt, deltaX(13:15, 1:N)*180/pi);
    grid on;
    title('Gyro bias error [deg/s]')
    legend(sprintf('x (%.3g)', sqrt(mean(((deltaX(13, 1:N))*180/pi).^2))),...
        sprintf('y (%.3g)', sqrt(mean(((deltaX(14, 1:N))*180/pi).^2))),...
        sprintf('z (%.3g)', sqrt(mean(((deltaX(15, 1:N))*180/pi).^2))))

    %suptitle('State estimate errors');
    if export_plots
        hgexport(f, 'figures/ga_2_sim_errors.eps', style, 'Format', 'eps');
    end
end

% error distance plot
if plot_error_distance
    figure(4); clf; hold on;
    subplot(2,1,1); hold on;
    plot((0:(N-1))*dt, sqrt(sum(deltaX(1:3, 1:N).^2,1)))
    plot((0:100:(N-1))*dt, sqrt(sum((xtrue(1:3, 100:100:N) - zGNSS(:, 1:GNSSk)).^2,1)))
    ylabel('Position error [m]')
    grid on;
    legend(sprintf('estimation error (%.3g)',sqrt(mean(sum(deltaX(1:3, 1:N).^2,1))) ),...
    sprintf('measurement error (%.3g)', sqrt(mean(sum((xtrue(1:3, 100:100:N) - zGNSS(:, 1:GNSSk)).^2,1)))));

    subplot(2,1,2);
    plot((0:(N-1))*dt, sqrt(sum(deltaX(4:6, 1:N).^2, 1)))
    ylabel('Speed error [m/s]');
    title(sprintf('RMSE: %.3g', sqrt(mean(sum(deltaX(4:6, 1:N).^2, 1)))));
    grid on;
end

%% CONSISTENCY
alpha = 0.05;
CI15 = chi2inv([alpha/2; 1 - alpha/2; 0.5], 15);
CI3 = chi2inv([alpha/2; 1 - alpha/2; 0.5], 3);

if plot_consistency
    f = figure(5); clf;
    subplot(4,2,1);
    plot((0:(N-1))*dt, log(NEES(1:N)));
    grid on;
    hold on;
    plot([0, N-1]*dt, log((CI15*ones(1,2)))', 'r--');
    insideCI = mean((CI15(1) <= NEES).* (NEES <= CI15(2)));
    title(sprintf('total NEES in log scale (%.3g%% inside %.3g%% CI)', 100*insideCI, 100*(1 - alpha)));

    subplot(4,2,2);
    plot((0:(N-1))*dt, log(NEESpos(1:N)));
    grid on;
    hold on;
    plot([0, N-1]*dt, log(CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESpos).* (NEESpos <= CI3(2)));
    title(sprintf('position NEES in log scale (%.3g%% inside %.3g%% CI)', 100*insideCI, 100*(1 - alpha)));

    subplot(4,2,3);
    plot((0:(N-1))*dt, log(NEESvel(1:N)));
    grid on;
    hold on;
    plot([0, N-1]*dt, log(CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESvel).* (NEESvel <= CI3(2)));
    title(sprintf('velocity NEES in log scale (%.3g%% inside %.3g%% CI)', 100*insideCI, 100*(1 - alpha)));

    subplot(4,2,4);
    plot((0:(N-1))*dt, log(NEESatt(1:N)));
    grid on;
    hold on;
    plot([0, N-1]*dt, log(CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESatt).* (NEESatt <= CI3(2)));
    title(sprintf('attitude NEES in log scale (%.3g%% inside %.3g%% CI)', 100*insideCI, 100*(1 - alpha)));

    subplot(4,2,5);
    plot((0:(N-1))*dt, log(NEESaccbias(1:N)));
    grid on;
    hold on;
    plot([0, N-1]*dt, log(CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESaccbias).* (NEESaccbias <= CI3(2)));
    title(sprintf('accelerometer bias NEES in log scale (%.3g%% inside %.3g%% CI)', 100*insideCI, 100*(1 - alpha)));

    subplot(4,2,6);
    plot((0:(N-1))*dt, log(NEESgyrobias(1:N)));
    grid on;
    hold on;
    plot([0, N-1]*dt, log(CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESgyrobias).* (NEESgyrobias <= CI3(2)));
    title(sprintf('gyro bias NEES in log scale (%.3g%% inside %.3g%% CI)', 100*insideCI, 100*(1 - alpha)));

    subplot(4,1,4)
    plot(0:(numel(NIS)-1), NIS);
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NIS).* (NIS <= CI3(2)));
    title(sprintf('NIS (%.3g%% inside %.3g%% CI)', 100*insideCI, 100*(1 - alpha)));
    
    if export_plots
        hgexport(f, 'figures/ga_2_sim_consistency.eps', style, 'Format', 'eps');
    end
end

% boxplot
if plot_boxplot
    figure(6)
    subplot(1,3,1)
    gaussCompare = sum(randn(3, numel(NIS)).^2, 1);
    boxplot([NIS', gaussCompare'],'notch','on',...
            'labels',{'NIS','gauss'})
    grid on
    subplot(1,3,2)
    gaussCompare15 = sum(randn(15, N).^2, 1);
    gaussCompare3 = sum(randn(3, N).^2, 1);
    boxplot([NEES(1:N)', gaussCompare15'],'notch', 'on', 'labels',{'NEES','gauss(15dim)'});
    grid on;
    subplot(1,3,3)
    boxplot([NEESpos(1:N)', NEESvel(1:N)', NEESatt(1:N)', NEESaccbias(1:N)', NEESgyrobias(1:N)', gaussCompare3'],...
        'notch', 'on', 'labels',{'NEESpos', 'NEESvel', 'NEESatt', 'NEESaccbias', 'NEESgyrobias', 'gauss(3dim)'})
    grid on;
end
