function expmat = ECE569_MatrixLog6(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a transformation matrix T in SE(3).
% Returns the corresponding se(3) representation of exponential 
% coordinates.
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% expmat = MatrixLog6(T)
% 
% Output:
% expc6 =
%         0         0         0         0
%         0         0   -1.5708    2.3562
%         0    1.5708         0    2.3562
%         0         0         0         0

[R, p] = ECE569_TransToRp(T);
omgmat = ECE569_MatrixLog3(R);

if isequal(omgmat, zeros(3))

    expmat = [zeros(3), p;
              0, 0, 0, 0];
else
    w_theta_vec = [omgmat(3,2); omgmat(1,3); omgmat(2,1)];
    theta = norm(w_theta_vec);
    
    % Get w_hat and w_hat_squared
    w_hat = omgmat / theta;
    w_hat_sq = w_hat * w_hat;
    
    % Calculate the 'A' matrix (which is G_inv * theta)
    % A = I - (theta/2)*w_hat + (1 - (theta/2)*cot(theta/2))*w_hat^2
    A_term = (1 - (theta / 2) * cot(theta / 2));
    A = eye(3) - (theta / 2) * w_hat + A_term * w_hat_sq;
    
    % Calculate the linear part (v*theta)
    v_theta_vec = A * p;
    
    % Assemble the final se(3) matrix [w_hat*theta, v*theta; 0, 0]
    expmat = [omgmat, v_theta_vec; 
              0, 0, 0, 0];
end

end