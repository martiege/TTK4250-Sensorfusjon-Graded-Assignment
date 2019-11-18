load simulatedSLAM;
K = numel(z);

do_plots = true;
plot_asso = false;
plot_results = true;
plot_nis = true; 
plot_nees = true; 
plot_movie = false;
export_plots = false;

%%
Q = diag([5e-1, 5e-1, 5e-2].^2); 
R = diag([1.5e-1, 1.5e-1].^2); % diag([1.2e-1, 1.2e-1].^2);
doAsso = true;
% 1 - chi2cdf([9, 25], 2)
% [0.0111, 3.7267e-6]
JCBBalphas = [1e-5, 1e-3]; % first is for joint compatibility, second is individual 
slam = EKFSLAM(Q, R, doAsso, JCBBalphas, zeros(2, 1), 0);

% allocate
xpred = cell(1, K);
Ppred = cell(1, K);
xhat = cell(1, K);
Phat = cell(1, K);
a = cell(1, K);

% init
xpred{1} = poseGT(:,1); % we start at the correct position for reference
Ppred{1} = zeros(3, 3); % we also say that we are 100% sure about that


figure(10); clf;
axAsso = gca;
N = K;
doAssoPlot = true; % set to true to se the associations that are done
tic
for k = 1:N
    prcdone(k, N, 'Come on and SLAM', 5); 
    
    [xhat{k}, Phat{k}, NIS(k), a{k}] =  slam.update(xpred{k}, Ppred{k}, z{k});
    NEESpose(k) = ((xhat{k}(1:3) - poseGT(:, k))' / Phat{k}(1:3, 1:3)) * (xhat{k}(1:3) - poseGT(:, k));
    RMSE(k) = sqrt(sum((xhat{k}(1:2) - poseGT(1:2, k)).^2, 1));
    if k < K
        [xpred{k + 1}, Ppred{k + 1}] = slam.predict(xhat{k}, Phat{k}, odometry(:, k));
    end
    
    % checks
    if size(xhat{k},1) ~= size(Phat{k},1)
        error('dimensions of mean and covariance do not match')
    end
    
    if doAssoPlot && k > 1 && do_plots && plot_asso % && any(a{k} == 0) % uncoment last part to only see new creations
        cla(axAsso); hold on;grid  on;
        zpred = reshape(slam.h(xpred{k}), 2, []);
        scatter(axAsso, z{k}(1, :), z{k}(2, :));
        scatter(axAsso, zpred(1, :), zpred(2, :));
        plot(axAsso, [z{k}(1, a{k}>0); zpred(1, a{k}(a{k}>0))], [z{k}(2, a{k}>0); zpred(2, a{k}(a{k}>0))], 'r', 'linewidth', 2)
        
        legend(axAsso, 'z', 'zbar') % , 'a')
        title(axAsso, sprintf('k = %d: %s', k, sprintf('%d, ',a{k})));
        %pause();
    end
end
toc

%% Do analysis and plotting
if do_plots
    plot_simulated_SLAM;
end