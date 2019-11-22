%% data loading
addpath('./victoria_park');
load('aa3_dr.mat');
load('aa3_lsr2.mat');
load('aa3_gpsx.mat');
timeOdo = time/1000; clear time;
timeLsr = double(TLsr)/1000; clear TLsr;
timeGps = timeGps/1000;
K = numel(timeOdo);
mK = numel(timeLsr);
gK = numel(timeGps);

do_plots = true;
export_plots = true;

%% Parameters
% the car parameters
car.L = 2.83; % axel distance
car.H = 0.76; % center to wheel encoder
car.a = 0.95; % laser distance in front of first axel
car.b = 0.5; % laser distance to the left of center

% the SLAM parameters
sigmas = [5e-1, 5e-1, 5e-2];
CorrCoeff = [1, 0, 0; 0, 1, 0.9; 0, 0.9, 1];
Q = diag(sigmas) * CorrCoeff * diag(sigmas); % (a bit at least) emprically found, feel free to change

R = diag([5e-2, 5e-2].^2);

alpha = 0.05;

JCBBalphas = [1e-5, 1e-5]; % first is for joint compatibility, second is individual 
sensorOffset = [car.a + car.L; car.b];
slam = EKFSLAM(Q, R, true, JCBBalphas, sensorOffset);

% allocate
xupd = zeros(3, mK);
a = cell(1, mK);
lmk = cell(1, mK);

% initialize TWEAK THESE TO BETTER BE ABLE TO COMPARE TO GPS
eta = [Lo_m(1); La_m(2); 38 * pi / 180]; % set the start to be relatable to GPS. 
P = zeros(3,3); % we say that we start knowing where we are in our own local coordinates
Pupd = cell(1, mK);

mk = 2; % first seems to be a bit off in timing
gk = 2;
t = timeOdo(1);
tic
N = 15000/10;

figure(1); clf;  hold on; grid on; axis equal;
ax = gca;
% cheeper to update plot data than to create new plot objects
lhPose = plot(ax, eta(1), eta(2), 'k');
shLmk = scatter(ax, nan, nan, 'rx');
shZ = scatter(ax, nan, nan, 'bo');
th = title(ax, 'start');

for k = 1:N
    if mk < mK && timeLsr(mk) <= timeOdo(k+1)
        dt = timeLsr(mk) - t;
        if  dt >= 0
            t = timeLsr(mk);
            odo = odometry(speed(k + 1), steering(k + 1), dt, car);
            [eta, P] = slam.predict(eta, P, odo);
        else
            error('negative time increment...')
        end
        z = detectTreesI16(LASER(mk,:));
        [eta, P, NIS(mk), a{k}] = slam.update(eta, P, z);
        %NIS(mk) = NIS(mk) / size(eta, 1); % scale NIS by dimension in order to better compare over time
        CI(:, mk) = chi2inv([alpha/2; 1 - alpha/2], 2* nnz(a{k}));
        nnz(a{k})
        xupd(:, mk) = eta(1:3);
        Pupd{mk} = P;
    
        mk = mk + 1;
        if do_plots
            lhPose.XData = [lhPose.XData, eta(1)];
            lhPose.YData = [lhPose.YData, eta(2)];
            shLmk.XData = eta(4:2:end);
            shLmk.YData = eta(5:2:end);
            if ~isempty(z)
                zinmap = rotmat2d(eta(3)) * (z(1,:).*[cos(z(2,:)); sin(z(2,:))] + slam.sensOffset) + eta(1:2);
                shZ.XData = zinmap(1,:);
                shZ.YData = zinmap(2,:);
            end

            th.String = sprintf('step %d, laser scan %d, landmarks %d, measurements %d, num new = %d', k, mk, (size(eta,1) - 3)/2, size(z,2), nnz(a{k} == 0));
            drawnow;
            pause(0.01)
        end
    end
    
    if k < K
        dt = timeOdo(k+1) - t;
        t = timeOdo(k+1);
        odo = odometry(speed(k+1), steering(k+1), dt, car);
        [eta, P] = slam.predict(eta, P, odo);
    end
    if mod(k, 50) == 0
        toc
        disp(k)
        tic
    end
end

%% Plots and analysis
if do_plots
    plot_real_SLAM;
end
