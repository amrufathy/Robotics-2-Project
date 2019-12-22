function [outputs] = MBP(args)
% Model Based Preview method
disp('MBP method')

import casadi.*;

q0 = args{2}; % initial configuration
jac = args{3}; % task jacobian
q = args{4}; % symbolic q vector
dq = args{5}; % symbolic dq vector
p = args{6}; % desired cartesian trajectory
dp = args{7}; % desired cartesian velocity
ddp = args{8}; % desired cartesian acceleration
ts = args{9}; % sampling timestep
M = args{10}; % inertia matrix of robot
c = args{11}; % coriolois & centrifugal terms of robot
djac = args{12}; % d(jac)/dt
fk = args{13}; % forward kinematics
S = args{15}; % factorization matrix for centrifugal terms
len = args{16}; % length of motion

% initial state
q_c(:, 1) = q0;
dq_c(:, 1) = [0; 0; 0];

pos_err = []; vel_err = []; % errors
Kp = 10 * eye(2); Kd = eye(2); % gains for controller

for i=1:len
    T = 100*ts;
    
    % current state (already computed)
    qi = q_c(:, i); dqi = dq_c(:, i);
    
    % next state
    qi1 = qi + dqi * T;
    
    % dynamic variables
    mqi = substitute(M, q, qi);
    jaci = substitute(jac, q, qi);
    cqi = substitute(c, [q dq], [qi dqi]);
    hqi = substitute(djac, [q dq], [qi dqi]) * dqi;
    fki = substitute(fk, q, qi);
    
    mqi1 = substitute(M, q, qi1);
    jaci1 = substitute(jac, q, qi1);
    hqi1 = ((jaci1 - jaci) / T) * dqi;
    sqi1 = substitute(S, [q dq], [qi1 dqi]);
    
    % errors
    e = p(:, i) - fki;
    de = dp(:, i) - jaci * dqi;
    pos_err = [pos_err, full(evalf(norm(e)))];
    vel_err = [vel_err, full(evalf(norm(de)))];
    
    % optimal ddq
    Q11 = mqi^2 + T^2 * sqi1' * sqi1; 
    Q12 = T * sqi1' * mqi1; Q21 = Q12; Q22 = mqi1^2;
    Q = [Q11 Q12; Q21 Q22];
    
    r11 = mqi * cqi + T * sqi1' * sqi1 * dqi;
    r21 = mqi1 * sqi1 * dqi;
    r = [r11; r21];
    
    A11 = jaci; A12 = zeros(size(jaci));
    A21 = jaci1 - jaci; A22 = jaci1;
    A = [A11 A12; A21 A22];
    
    b11 = (ddp(:, i) + Kd * de + Kp * e) - hqi; b21 = ddp(:, i + T/ts) - hqi1;
    b = [b11; b21];
    
    A_winv = inv(Q) * A' * inv(A * inv(Q) * A');
    
    ddqi = full(evalf(...
        A_winv * b - (eye(6) - A_winv * A) * inv(Q) * r...
    ));
    
    ddqi = ddqi(1:3);
    ddq_c(:, i) = ddqi;
    
    % min norm torque
    t_opt = full(evalf(mqi * ddqi + cqi));
    trq_c(:, i) = t_opt;
    
    % next state (using euler integration)
    if i ~= len
        dq_c(:, i + 1) = dqi + ddqi * ts;
        q_c(:, i + 1) = qi + dqi * ts + 0.5 * ddqi * ts^2;
    end
    
end

outputs = {q_c, dq_c, ddq_c, trq_c, pos_err, vel_err};

end