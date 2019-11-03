function R = quat2rotmat(quat)  
    eta = quat(1);
    epsilon = quat(2:4);
    S = crossProdMat(epsilon);
    
    R = eye(3) + 2 * eta * S + 2 * S * S;
end