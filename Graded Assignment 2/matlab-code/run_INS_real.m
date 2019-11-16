load task_real;
IMUTs = diff(timeIMU);
dt = mean(IMUTs);
K = size(zAcc,2);
K_start = 1;

loading_print = true;
plotting             = true; % plot
export_plots         = true;

%% Measurement noise
% GNSS Position measurement
%GNSSaccMean = mean(GNSSaccuracy);
%p_std = 0.5*ones(3,1) * GNSSaccMean;
%RGNSS = diag(p_std.^2);
p_std_gain = 0.5;

% accelerometer
g = 9.80665; 

qA = (1e-4 * g)^2; % accelerometer measurement noise covariance
qAb = (0.05e-3 * g)^2; % accelerometer bias driving noise covariance
pAcc = 1e-16; % accelerometer bias reciprocal time constant

qG = (deg2rad(1e0) / (60^2))^2; % gyro measurement noise covariance
qGb = (deg2rad(0.5) / (60^2))^2;  % gyro bias driving noise covariance
pGyro = 1e-16; % gyro bias reciprocal time constant

%% Estimator
eskf = ESKF(qA, qG, qAb, qGb, pAcc, pGyro);
eskf.Sa = S_a; % set the accelerometer correction matrix
eskf.Sg = S_g; % set the gyro correction matrix
%% Allocate
xest = zeros(16, K);
Pest = zeros(15, 15, K);

xpred = zeros(16, K);
Ppred = zeros(15, 15, K);

%% initialize
xpred(1:3, 1) = [0, 0, 0]'; % starting 5 meters above ground
%xpred(4:6, 1) = [0.0173; -0.0129; 0.1983]; %
xpred(7, 1) = cosd(45); % nose to east, right to south and belly down.
xpred(10, 1) = sind(45);

Ppred(1:3, 1:3, 1) = 10^2*eye(3); 
Ppred(4:6, 4:6, 1) = 3^2*eye(3);
Ppred(7:9, 7:9, 1) = (pi/30)^2 * eye(3); % error rotation vector (not quat)
Ppred(10:12, 10:12, 1) = 0.05^2 * eye(3);
Ppred(13:15, 13:15, 1) = (2e-5)^2 * eye(3);

%% run
N = K;
GNSSk = 1;
for k = K_start:N
    t = timeIMU(k);
    
    if loading_print
        prcdone(k,N,'ESKF',1);
    end
    
    if timeGNSS(GNSSk) < t
        p_std = p_std_gain * GNSSaccuracy(GNSSk) * ones(3, 1);
        RGNSS = diag(p_std.^2);
        NIS(GNSSk) = eskf.NISGNSS(xpred(:, k), Ppred(:, :, k), zGNSS(:, GNSSk), RGNSS, leverarm);
        [xest(:, k), Pest(:, :, k)] = eskf.updateGNSS(xpred(:, k), Ppred(:, :, k), zGNSS(:, GNSSk), RGNSS, leverarm);
        GNSSk = GNSSk + 1;
    else % no updates so estimate = prediction
        xest(:, k) = xpred(:, k);
        Pest(:, :, k) = Ppred(:, :, k);
    end

    if k < K
        [xpred(:, k+1),  Ppred(:, :, k+1)] = eskf.predict(xest(:, k), Pest(:, :, k), zAcc(:, k), zGyro(:, k), dt);
    end  
end
%% plots
if plotting
    plotting_script_real
end