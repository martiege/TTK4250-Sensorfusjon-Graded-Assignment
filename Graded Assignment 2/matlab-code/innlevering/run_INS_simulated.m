load task_simulation.mat;
dt = mean(diff(timeIMU));
steps = size(zAcc,2);

loading_print        = true; % print percentage of loop completed
plotting             = true; % plot
plot_path            = true; % figure 1
plot_state_estimates = true; % figure 2
plot_state_error     = false; % figure 3
plot_error_distance  = false; % figure 4
plot_consistency     = true; % figure 5
plot_boxplot         = false; % figure 6

%% Measurement noise
% GNSS Position  measurement
p_std = 4e-1 * [1 1 1]'; % Measurement noise
RGNSS = diag(p_std.^2);

% accelerometer
qA = (4e-2)^2; % accelerometer measurement noise covariance
qAb = (1e-3)^2; % accelerometer bias driving noise covariance
pAcc = 1e-8; % accelerometer bias reciprocal time constant

qG = (8e-4)^2; % gyro measurement noise covariance
qGb = (1e-6)^2;  % gyro bias driving noise covariance
pGyro = 1e-8; % gyro bias reciprocal time constant

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

Ppred(1:3, 1:3, 1) = 1e-3*eye(3); 
Ppred(4:6, 4:6, 1) = 1e-3*eye(3);
Ppred(7:9, 7:9, 1) = 1e-5*eye(3); % error rotation vector (not quat)
Ppred(10:12, 10:12, 1) = 1e-2*eye(3);
Ppred(13:15, 13:15, 1) = 1e-6*eye(3);
%% run
N = 90000;
GNSSk = 1;
for k = 1:N
    if loading_print
        prcdone(k,N,'ESKF',10);
    end
    
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
if plotting
    plotting_script_simulated
end