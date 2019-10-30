function comp = composition(xnom, deltaX)
%% Attitude-friendly composition
    % Adds x and deltaX, with special case for quaternions. 
    % Quaternion is normalized. 
    %
    % x (16 x 1): nominal state
    % deltaX (15 x 1): estimated (updated from a measurement) error state
    % 
    % comp (16 x 1): composition of x and deltaX

    quaternion = quatProd(xnom(7:10), [1; deltaX(7:9)/2]);
    
    comp = [xnom(1:6) + deltaX(1:6); 
            quaternion / norm(quaternion); 
            xnom(11:16) + deltaX(10:15)];

end

