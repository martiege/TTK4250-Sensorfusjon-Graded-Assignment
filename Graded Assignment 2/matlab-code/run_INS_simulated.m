load task_simulation.mat;
dt = mean(diff(timeIMU));
steps = size(zAcc,2);

plot_path            = true; % figure 1
plot_state_estimates = true; % figure 2
plot_state_error     = true; % figure 3
plot_error_distance  = true; % figure 4
plot_consistency     = true; % figure 5
plot_boxplot         = true; % figure 6

%% Measurement noise
% GNSS Position  measurement
p_std = 1e-1 * [1 1 1]'; % Measurement noise
RGNSS = diag(p_std.^2);

% accelerometer
qA = (1e-2)^2; % accelerometer measurement noise covariance
qAb = (1e-4)^2; % accelerometer bias driving noise covariance
pAcc = 0 * 1e-6; % accelerometer bias reciprocal time constant

qG = (1e-2)^2; % gyro measurement noise covariance
qGb = (1e-3)^2;  % gyro bias driving noise covariance
pGyro = 0 * 1e-5; % gyro bias reciprocal time constant

%% Estimator
eskf = ESKF(qA, qG, qAb, qGb, pAcc, pGyro);
eskf.Sa = S_a; % set the accelerometer correction matrix
eskf.Sg = S_g; % set the gyro correction matrix

%% Allocate
xest = zeros(16, steps);
Pest = zeros(15, 15, steps);

xpred = zeros(16, steps);
Ppred = zeros(15, 15, steps);

%% initialize
xpred(1:3, 1) = [0, 0, -5]'; % starting 5 meters above ground
xpred(4:6, 1) = [20, 0, 0]'; % starting at 20 m/s due north
xpred(7, 1) = 1; % no initial rotation: nose to north, right to East and belly down.

Ppred(1:3, 1:3, 1) = 1e-2*eye(3); 
Ppred(4:6, 4:6, 1) = 1e-2*eye(3);
Ppred(7:9, 7:9, 1) = 1e-5*eye(3); % error rotation vector (not quat)
Ppred(10:12, 10:12, 1) = 1e-3*eye(3);
Ppred(13:15, 13:15, 1) = 1e-5*eye(3);

%% run
N = 90000/10;
GNSSk = 1;
for k = 1:N
    prcdone(k,N,'ESKF',10);
    
    if  timeIMU(k) >= timeGNSS(GNSSk)
        NIS(GNSSk) = eskf.NISGNSS(xpred(:, k), Ppred(:, :, k), zGNSS(:, GNSSk), RGNSS, leverarm);
        [xest(:, k), Pest(:, :, k)] = eskf.updateGNSS(xpred(:, k), Ppred(:, :, k), zGNSS(:, GNSSk), RGNSS, leverarm);
        GNSSk = GNSSk  + 1;
    else % no updates so estimate = prediction
        xest(:, k) = xpred(:, k);
        Pest(:, :, k) = Ppred(:, :, k);
    end
    
    deltaX(:, k) = eskf.deltaX(xest(:, k), xtrue(:, k));
    [NEES(:, k), NEESpos(:, k), NEESvel(:, k), NEESatt(:, k), NEESaccbias(:, k), NEESgyrobias(:, k)] = ...
        eskf.NEES(xest(:, k), Pest(:, :, k), xtrue(:, k));
    
    if k < N
        [xpred(:, k+1),  Ppred(:, :, k+1)] = eskf.predict(xest(:, k), Pest(:, :, k), zAcc(:, k), zGyro(:, k), dt);
    end
end

GNSSk = GNSSk - 1;    

%% plots
if plot_path
    figure(1);
    clf;
    plot3(xest(2, 1:N), xest(1, 1:N), -xest(3, 1:N));
    hold on;
    plot3(zGNSS(2, 1:GNSSk), zGNSS(1, 1:GNSSk), -zGNSS(3, 1:GNSSk))
    grid on; 
    xlabel('East [m]')
    ylabel('North [m]')
    zlabel('Altitude [m]')
end
% state estimate plot
eul = quat2eul(xest(7:10, :));
eul_true = quat2eul(xtrue(7:10, :));

if plot_state_estimates
    figure(2); clf; hold on;

    subplot(5,1,1);
    plot((0:(N-1))*dt, xest(1:3, 1:N))
    grid on;
    ylabel('NED position [m]')
    legend('North', 'East', 'Down')

    subplot(5,1,2);
    plot((0:(N-1))*dt, xest(4:6, 1:N))
    grid on;
    ylabel('Velocitites [m/s]')
    legend('North', 'East', 'Down')

    subplot(5,1,3);
    plot((0:(N-1))*dt, eul(:, 1:N)*180/pi)
    hold on
    %plot((0:(N-1))*dt, euler_out(:, 1:N)*180/pi)
    grid on;
    ylabel('euler angles [deg]')
    legend('\phi', '\theta', '\psi')

    subplot(5, 1, 4)
    plot((0:(N-1))*dt, xest(11:13, 1:N))
    grid on;
    ylabel('Accl bias [m/s^2]')
    legend('x', 'y', 'z')
    subplot(5, 1, 5)
    plot((0:(N-1))*dt, xest(14:16, 1:N)*180/pi * 3600)
    grid on;
    ylabel('Gyro bias [deg/h]')
    legend('x', 'y', 'z')

    %suptitle('States estimates');
end

% state error plots
if plot_state_error
    figure(3); clf; hold on;

    subplot(5,1,1);
    plot((0:(N-1))*dt, deltaX(1:3,1:N))
    grid on;
    ylabel('NED position error [m]')
    legend(sprintf('North (%.3g)', sqrt(mean(deltaX(1, 1:N).^2))),...
        sprintf('East (%.3g)', sqrt(mean(deltaX(2, 1:N).^2))),...
        sprintf('Down (%.3g)', sqrt(mean(deltaX(3, 1:N).^2))))

    subplot(5,1,2);
    plot((0:(N-1))*dt, deltaX(4:6, 1:N))
    grid on;
    ylabel('Velocitites error [m/s]')
    legend(sprintf('North (%.3g)', sqrt(mean(deltaX(4, 1:N).^2))),...
        sprintf('East (%.3g)', sqrt(mean(deltaX(5, 1:N).^2))),...
        sprintf('Down (%.3g)', sqrt(mean(deltaX(6, 1:N).^2))))

    subplot(5,1,3);
    plot((0:(N-1))*dt, wrapToPi(eul(:, 1:N) - eul_true(:, 1:N))*180/pi)
    grid on;
    ylabel('euler angles error [deg]')
    legend(sprintf('\\phi (%.3g)', sqrt(mean((eul(1, 1:N) - eul_true(1, 1:N)).^2))),...
        sprintf('\\theta (%.3g)', sqrt(mean((eul(2, 1:N) - eul_true(2, 1:N)).^2))),...
        sprintf('\\psi (%.3g)', sqrt(mean((eul(3, 1:N) - eul_true(3, 1:N)).^2))))

    subplot(5, 1, 4)
    plot((0:(N-1))*dt, deltaX(10:12, 1:N))
    grid on;
    ylabel('Accl bias error [m/s^2]')
    legend(sprintf('x (%.3g)', sqrt(mean(deltaX(1, 1:N).^2))),...
        sprintf('y (%.3g)', sqrt(mean(deltaX(11, 1:N).^2))),...
        sprintf('z (%.3g)', sqrt(mean(deltaX(12, 1:N).^2))))

    subplot(5, 1, 5)
    plot((0:(N-1))*dt, deltaX(13:15, 1:N)*180/pi);
    grid on;
    ylabel('Gyro bias error [deg/s]')
    legend(sprintf('x (%.3g)', sqrt(mean(((deltaX(13, 1:N))*180/pi).^2))),...
        sprintf('y (%.3g)', sqrt(mean(((deltaX(14, 1:N))*180/pi).^2))),...
        sprintf('z (%.3g)', sqrt(mean(((deltaX(15, 1:N))*180/pi).^2))))

    %suptitle('States estimate errors');
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
    figure(5); clf;
    subplot(7,1,1);
    plot((0:(N-1))*dt, NEES(1:N));
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI15*ones(1,2))', 'r--');
    insideCI = mean((CI15(1) <= NEES).* (NEES <= CI15(2)));
    title(sprintf('total NEES (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));

    subplot(7,1,2);
    plot((0:(N-1))*dt, NEESpos(1:N));
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESpos).* (NEESpos <= CI3(2)));
    title(sprintf('position NEES (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));

    subplot(7,1,3);
    plot((0:(N-1))*dt, NEESvel(1:N));
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESvel).* (NEESvel <= CI3(2)));
    title(sprintf('velocity NEES (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));

    subplot(7,1,4);
    plot((0:(N-1))*dt, NEESatt(1:N));
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESatt).* (NEESatt <= CI3(2)));
    title(sprintf('attitude NEES (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));

    subplot(7,1,5);
    plot((0:(N-1))*dt, NEESaccbias(1:N));
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESaccbias).* (NEESaccbias <= CI3(2)));
    title(sprintf('accelerometer bias NEES (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));

    subplot(7,1,6);
    plot((0:(N-1))*dt, NEESgyrobias(1:N));
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NEESgyrobias).* (NEESgyrobias <= CI3(2)));
    title(sprintf('gyro bias NEES (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));

    subplot(7,1,7)
    plot(0:(numel(NIS)-1), NIS);
    grid on;
    hold on;
    plot([0, N-1]*dt, (CI3*ones(1,2))', 'r--');
    insideCI = mean((CI3(1) <= NIS).* (NIS <= CI3(2)));
    title(sprintf('NIS (%.3g%% inside %.3g%% confidence intervall)', 100*insideCI, 100*(1 - alpha)));
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