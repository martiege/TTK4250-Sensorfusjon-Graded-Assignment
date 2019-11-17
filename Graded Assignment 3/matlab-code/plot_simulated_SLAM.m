style = hgexport('factorystyle');
style.bounds = 'tight';
style.Format = 'eps';
style.Width = 6;
style.Height = 2.5;
style.Resolution = 300;
style.Units = 'inch';
style.FixedFontSize = 12;

% plotting
if plot_results
    f = figure(3);
    k = N;
    clf;
    %subplot(1,2,1);
    hold on;

    scatter(landmarks(1,:), landmarks(2,:), 'r^')
    scatter(xhat{k}(4:2:end), xhat{k}(5:2:end), 'b.')

    lh1 = plot(poseGT(1, 1:k), poseGT(2,1:k), 'r', 'DisplayName', 'Ground Truth');
    lh2 = plot(cellfun(@(x) x(1), xhat), cellfun(@(x) x(2), xhat), 'b', 'DisplayName', 'Estimated');

    el = ellipse(xhat{k}(1:2),Phat{k}(1:2,1:2),5,200);
    plot(el(1,:),el(2,:),'b');

    for ii=1:((size(Phat{k}, 1)-3)/2)
       rI = squeeze(Phat{k}(3+[1,2]+(ii-1)*2,3+[1,2]+(ii-1)*2));
       el = ellipse(xhat{k}(3 + (1:2) + (ii-1)*2),rI,5,200);
       plot(el(1,:),el(2,:),'b');
    end

    axis equal;
    title('Estimated trajectory and landmarks vs. ground truth')
    legend([lh1, lh2])
    grid on;
    
    if export_plots
        hgexport(f,'figures/ga_3_sim_trajectory.eps',style,'Format','eps');
    end
end
% subplot(1,2,2);
% hold on;
% % funF = @(delta) sum([poseGT(1:2,k) - rotmat2d(delta(3)) * xhat(1:2,k) - delta(1:2), landmarks - rotmat2d(delta(3)) * reshape(xhat(4:end,k),2,[]) - delta(1:2)].^2 ,[1,2]);
% % funS = @(delta) sum([poseGT(1:2,1) - delta(1:2), landmarks - rotmat2d(delta(3)) * reshape(xhat(4:end,1),2,[]) - delta(1:2)].^2 ,[1,2]);
% funF = @(delta) sum([poseGT(1:2,k) - rotmat2d(delta(3)) * xhat{k}(1:2) - delta(1:2), landmarks(:, a) - rotmat2d(delta(3)) * reshape(xhat{k}(4:end),2,[]) - delta(1:2)].^2 ,[1,2]);
% funS = @(delta) sum([poseGT(1:2,1) - delta(1:2), landmarks - rotmat2d(delta(3)) * reshape(xhat{1}(4:end),2,[]) - delta(1:2)].^2 ,[1,2]);
% % deltaXFinal = fminunc(funF, zeros(3,1))
% % deltaXStart = fminunc(funS, zeros(3,1))
% 
% Rot = rotmat2d(deltaXFinal(3));
% %xcomp = [Rot, zeros(2,1); zeros(1, 2), 1]*xhat(1:3, :) + deltaXFinal;
% for k = 1:K
%     xcomp(:, k) = [Rot, zeros(2,1); zeros(1, 2), 1]*xhat{k}(1:3) + deltaXFinal;
% end
% %mcomp = Rot * reshape(xhat(4:end,k),2,[]) + deltaXFinal(1:2);
% mcomp = Rot * reshape(xhat{k}(4:end),2,[]) + deltaXFinal(1:2);
% 
% scatter(landmarks(1,:), landmarks(2,:), 'r^')
% scatter(mcomp(1,:), mcomp(2,:), 'b.')
% plot(poseGT(1, 1:k), poseGT(2,1:k), 'r');
% plot(xcomp(1,:), xcomp(2,:), 'b');
% 
% %el = ellipse(xcomp(1:2,k), Rot * squeeze(Phat(1:2,1:2,k)) * Rot',5,200);
% el = ellipse(xcomp(1:2,k), Rot * squeeze(Phat{k}(1:2,1:2)) * Rot',5,200);
% plot(el(1,:),el(2,:),'b');

% for ii=1:m
%    %rI = squeeze(Rot * Phat(3+[1,2]+(ii-1)*2,3+[1,2]+(ii-1)*2,k) * Rot'); 
%    rI = squeeze(Rot * Phat{k}(3+[1,2]+(ii-1)*2,3+[1,2]+(ii-1)*2) * Rot'); 
%    el = ellipse(mcomp(:,ii),rI,5,200); 
%    plot(el(1,:),el(2,:),'b');
% end
% axis equal;
% grid on;
% title(sprintf('transformed: x = %0.2fm, y = %0.2fm , \\theta = %0.2fdeg',deltaXFinal(1:2),deltaXFinal(3)*180/pi))


%% consistency: what to do with variable measurement size..?
alpha = 0.05;
ANIS = mean(NIS)
ACI = chi2inv([alpha/2; 1 - alpha/2], 1)/N % NOT CORRECT NOW
CI = chi2inv([alpha/2; 1 - alpha/2], 1); % NOT CORRECT NOW
warning('These consistency intervals have wrong degrees of freedom')

if plot_nis
    f = figure(5); clf;
    hold on;
    plot(1:N, NIS(1:N));
    insideCI = mean((CI(1) < NIS) .* (NIS <= CI(2)))*100;
    plot([1, N], (CI*ones(1, 2))','r--');
    title(sprintf('NIS over time, with %0.1f%% inside %0.1f%% CI', insideCI, (1-alpha)*100));
    grid on;
    ylabel('NIS');
    xlabel('time');
    
    if export_plots
        hgexport(f,'figures/ga_3_sim_NIS.eps',style,'Format','eps');
    end
end

if plot_nees
    f = figure(6); clf;
    hold on;
    plot(NEESpose); grid on; hold on;
    ylabel('NEES in pose');
    insideCI = mean((CI(1) < NEESpose) .* (NEESpose <= CI(2)))*100;  % Probably not correct
    plot([1, N], (CI*ones(1, 2))','r--');
    title(sprintf('NEES over time, with %0.1f%% inside %0.1f%% CI', insideCI, (1-alpha)*100));
    
    if export_plots
        hgexport(f,'figures/ga_3_sim_NEES.eps',style,'Format','eps');
    end
end

%% run a movie
if plot_movie
    pauseTime = 0.05;
    fig = figure(4);
    ax = gca;
    for k = 1:N
        cla(ax); hold on;
        scatter(ax, landmarks(1,:), landmarks(2,:), 'r^')
        scatter(ax, xhat{k}(4:2:end), xhat{k}(5:2:end), 'b*')
        plot(ax, poseGT(1, 1:k), poseGT(2,1:k), 'r-o','markerindices',10:10:k);
        plot(ax, cellfun(@(x) x(1), xhat(1:k)), cellfun(@(x) x(2), xhat(1:k)), 'b-o','markerindices',10:10:k);

        if k > 1 % singular cov at k = 1
            el = ellipse(xhat{k}(1:2),Phat{k}(1:2,1:2),5,200);
            plot(ax,el(1,:),el(2,:),'b');
        end

        for ii=1:((size(Phat{k}, 1)-3)/2)
           rI = squeeze(Phat{k}(3+[1,2]+(ii-1)*2,3+[1,2]+(ii-1)*2)); 
           el = ellipse(xhat{k}(3 + (1:2) + (ii-1)*2),rI,5,200);
           plot(ax, el(1,:),el(2,:),'b');
        end

        title(ax, sprintf('k = %d',k))
        grid(ax, 'on');
        pause(pauseTime);
    end
end