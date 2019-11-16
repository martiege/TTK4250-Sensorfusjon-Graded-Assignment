classdef EKFSLAM
    properties
        Q
        R
        doAsso
        alpha
        sensOffset
        doChecks
    end
    methods
        function obj = EKFSLAM(Q, R, doAsso, alphas, sensorOffset, doChecks)
            obj.Q = Q;
            obj.R = R;
            
            if nargin < 3
                doAsso = false;
            end
            obj.doAsso = doAsso;
            
            if nargin < 4
                alphas = [0.001, 0.0001];
            end
            obj.alpha = alphas;
            
            if nargin < 5
                sensorOffset = zeros(2,1);
            end
            
            if nargin < 6
                doChecks = false; 
            end
            obj.doChecks = doChecks; 
            
            obj.sensOffset = sensorOffset(:);
        end
        
        function xpred = f(~, x, u)
            % equation 11.7
            s_psi = sin(x(3)); 
            c_psi = cos(x(3)); 
            xpred = [x(1) + u(1) * c_psi - u(2) * s_psi;
                     x(2) + u(1) * s_psi + u(2) * c_psi;
                     x(3) + u(3)];
        end
        
        function Fx = Fx(obj, x, u)
            % equation 11.13
            s_psi = sin(x(3)); 
            c_psi = cos(x(3));
            Fx = [1, 0, -u(1) * s_psi - u(2) * c_psi; 
                  0, 1,  u(1) * c_psi - u(2) * s_psi; 
                  0, 0,  1]; 
            
            % check that jacobian is correct, remove for speed
            if obj.doChecks
                if norm(Fx - jacobianFD(@(X) obj.f(X, u), x, 1e-5), 'fro') > 1e-3
                    error('some error in pred Jac')
                end
            end
        end
        
        function Fu = Fu(obj, x, u)
            % equation 11.14
            s_psi = sin(x(3)); 
            c_psi = cos(x(3));
            Fu = [c_psi, -s_psi, 0; 
                  s_psi,  c_psi, 0; 
                      0,      0, 1]; 
              
            % check that the jacobian is correct, remove for speed
            if obj.doChecks
                if norm(Fu - jacobianFD(@(U) obj.f(x, U), u, 1e-5), 'fro') > 1e-3
                    jacobianFD(@(U) obj.f(x, U), u, 1e-5)
                    error('some error in pred Jac')
                end
            end
        end
        
        function [etapred, P] =  predict(obj, eta, P, zOdo)
            x = eta(1:3); % pose
            m = eta(4:end); % map
            
            xpred = obj.f(x, zOdo);
            Fx = obj.Fx(x, zOdo); 
            Fu = obj.Fu(x, zOdo); 
            
            % in place for performance 
            P(1:3, 1:3) = Fx * P(1:3, 1:3) * Fx' + Fu * obj.Q * Fu'; % resize Q?
            P(1:3, 4:end) = Fx * P(1:3, 4:end); 
            P(4:end, 1:3) = P(4:end, 1:3) * Fx'; 
            
            % concatenate pose and landmarks again
            etapred = [xpred; m];
            
            % check that the covariance makes sense
            if obj.doChecks
                if any(eig(P) <= 0) % costly, remove when tested
                    warn('EKFpredict got cov not PSD')
                end
            end
        end
        
        function zpred = h(obj, eta)
            x = eta(1:3); % pose 
            m = reshape(eta(4:end), 2 ,[]); % map (2 x m now)
            
            Rot = rotmat2d(-x(3)); % rot from world to body
            
            % cartesian measurement in world
            z_c = m - x(1:2) - Rot' * obj.sensOffset;
            
            % in body
            z_b = Rot * z_c; 
            
            % polar
            zpred = [sqrt(sum(z_b.^2, 1)); atan2(z_b(2, :), z_b(1, :))]; 
            
            % make column again
            zpred = zpred(:); 
        end
        
        function H = H(obj, eta)
            x = eta(1:3); % pose
            m = reshape(eta(4:end), 2 ,[]); % map
            
            numM = size(m, 2);  % number of landmarks
            
            Rot = rotmat2d(x(3));
            
            m_minus_rho = m - x(1:2); 
            z_c = m_minus_rho - Rot * obj.sensOffset;
            
%             zpred = reshape(obj.h(eta), 2, []);
%             zr = zpred(1,:);
            
            Rpihalf = [0, -1; 1, 0]; 
            
            % allocate
            Hx = zeros(2 * numM, 3); % pose columns
            Hm = zeros(2 * numM, 2 * numM); % map columns (the rest)
            
            for i = 1:numM
                inds = 2*(i - 1) + [1; 2];
                
                jac_z_b = [-eye(2), -Rpihalf * m_minus_rho(:, i)];
                
                Hx(inds(1), :) = z_c(:, i)' / norm(z_c(:, i)) * jac_z_b; % jac z_r 
                Hx(inds(2), :) = z_c(:, i)' * Rpihalf' / (norm(z_c(:, i))^2) * jac_z_b; % jac z_phi
            
                % smack on a minus if wrong Hmmmmmm
                Hm(inds, inds) = -Hx(inds, 1:2); % should be negative of the two first colums of Hx
            end
        
            % concatenate the H matrix
            H = [Hx, Hm];
            
            % check that it is done correctly, remove for speed
            if obj.doChecks
                % TODO: Fix this
                if norm(H - jacobianFD(@(X) obj.h(X), eta, 1e-5), 'fro') > 1e-3
                    error('some error in meas Jac')
                end
            end
        end
        
        function [etaadded, Padded] = addLandmarks(obj, eta, P, z)
            n = size(P, 1);
            numLmk = numel(z)/2;
            
            % allocate
            lmnew = zeros(size(z));
            Gx = zeros(numLmk * 2, 3);
            Rall = zeros(numLmk * 2, numLmk * 2);
            
            Rpsipluspihalf = rotmat2d(eta(3) + pi/2); 

            for j = 1:numLmk
                % find indeces and the relevant measurement
                inds = 2 * (j - 1) + [1, 2];
                zj = z(inds);
                
                rot_meas = rotmat2d(zj(2)); 
                rot_body = rotmat2d(eta(3));

                lmnew(inds) = rot_body * (pol2cart(zj) + obj.sensOffset) + eta(1:2); % mean
                Gx(inds, :) = [eye(2), zj(1) * [-sin(zj(2) + eta(3)); cos(zj(2) + eta(3))] + Rpsipluspihalf * obj.sensOffset]; % jac h^-1 wrt. x
                Gz =  rot_meas * rot_body * diag([1, zj(1)]); % jac h^-1 wrt. z
                Rall(inds, inds) = Gz * obj.R * Gz'; % the linearized measurement noise

            end
            
            % augment state
            etaadded = [eta; lmnew];
            
            % add covariances
            Padded = blkdiag(P, Gx * P(1:3, 1:3) * Gx' + Rall);
            Padded((n+1):end, 1:n) = Gx*P(1:3, :);
            Padded(1:n, (n+1):end) = P(:, 1:3)*Gx';

            % sanity check, remove for speed
            if obj.doChecks
                if any(eig(Padded) <= 0) % costly, remove when tested
                    warning('EKFupdate got cov not PSD after adding a landmark');
                end
            end
        end
        
        function [z, zpred, H, S, a] = associate(obj, z, zpred, H, S)
            if obj.doAsso
                % associate
                a = JCBB(z, zpred, S, obj.alpha(1), obj.alpha(2));

                % extract associated measurements
                zinds = false(size(z));
                zinds(1:2:end) = a > 0;
                zinds(2:2:end) = zinds(1:2:end);
                z = z(zinds);

                % extract and rearange predicted measurements and cov
                zbarinds = reshape([2*a(a>0) - 1, 2*a(a>0)]', [], 1);
                zpred = zpred(zbarinds);
                S = S(zbarinds, zbarinds);
                H = H(zbarinds, :);
            % else  % if no association is to be done, assume that all measurements are there and in order
            end
        end
        
        function [etaupd, Pupd, NIS, a] = update(obj, eta, P, z)    
            numLmk = (numel(eta) - 3)/2; % number of landmarks
            if numLmk > 0
                % prediction and innovation covariance
                zpred = obj.h(eta);
                H = obj.H(eta); 
                S = H * P * H' + kron(eye(numLmk), obj.R); 
                z = z(:); % vectorize
                
                % perform data association if it is asked for
                [za, zpred, H, S, a] = obj.associate(z, zpred, H, S);

                % create the associated innovation
                v = za(:) - zpred;
                v(2:2:end) = wrapToPi(v(2:2:end)); % angles are in [-pi, pi]

                % Kalman update
                W = P * H' / S; 
                % W = P * H' * inv(S); 
                etaupd = eta + W * v; 
                NIS = v' / S * v; 
                Pupd = (eye(size(P)) - W * H) * P; 
                
                % sanity check, remove for speed
                if obj.doChecks
                    if any(eig(Pupd) <= 0) % costly, remove when tested
                        warn('EKFupdate got cov not PSD');
                    end
                end
            else % all measurements are new landmarks
                a = zeros(size(z, 2), 1);
                z = z(:);
                NIS = 0;
                etaupd = eta;
                Pupd = P;
            end
            
            % create new landmarks if any is available
            if obj.doAsso
                isNewLmk = (a == 0);
                if any(isNewLmk)
                    % extract unassociated measurements
                    zNewInds = false(size(z));
                    zNewInds(1:2:end) = isNewLmk;
                    zNewInds(2:2:end) = isNewLmk;
                    znew = z(zNewInds);
                    
                    % create new landmarks
                    [etaupd, Pupd] = obj.addLandmarks(eta, P, znew);
                end
            end  
        end
    end
end
