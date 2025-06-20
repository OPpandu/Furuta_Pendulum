% Constants
mp = 1;     % mass of pendulum
d = 0.5;    % distance of COM of pendulum from pivot
Ip = 1;     % MOI of pendulum about COM
Bp = 0;     % damping coefficient of pendulum
l = 1;      % length of arm
g = 9.81;   % gravity

A_coeff = Ip + mp * d^2;

% State Space Matrix A (4x4)
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     (mp*d*g)/A_coeff, 0, -Bp/A_coeff, 0;
     0, 0, 0, 0];

% Input Matrix B (4x1)
B = [0;
     0;
     (mp*l*d)/A_coeff;
     1];

% Display system matrices
disp('System Matrix A:');
disp(A);
disp('Input Matrix B:');
disp(B);

% Q matrix (4x4) - should match the number of states
Q = [100000   0   0   0;
     0   .1 0   0;
     0   0   1   0;
     0   0   0   1];

% R matrix (scalar for single input)
R = 1;

% Check controllability
if rank(ctrb(A, B)) == size(A, 1)
    disp('System is controllable!');
    
    % Design LQR controller
    [K, S, P] = lqr(A, B, Q, R);
    
    disp('LQR Gain Matrix K:');
    disp(K);
    
    % Closed-loop system matrix
    A_cl = A - B*K;
    
    disp('Closed-loop poles:');
    poles = eig(A_cl);
    disp(poles');
    
    % Check stability
    if all(real(poles) < 0)
        disp('Closed-loop system is stable!');
    else
        disp('Warning: Closed-loop system is unstable!');
    end
    
    % Display numerical values
    fprintf('\nNumerical values:\n');
    fprintf('A_coeff = %.4f\n', A_coeff);
    fprintf('mp*d*g/A_coeff = %.4f\n', (mp*d*g)/A_coeff);
    fprintf('mp*l*d/A_coeff = %.4f\n', (mp*l*d)/A_coeff);
    
else
    error('System is uncontrollable! Re-check A/B matrices.');
end

% Optional: Plot step response of closed-loop system
figure;
sys_cl = ss(A_cl, B, eye(4), 0);
step(sys_cl);
title('Closed-loop Step Response');
grid on;