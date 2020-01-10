function [outputs] = MTNB(args)
% Min Torque Norm - with bounds method
disp('MTNB method')

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
tb = args{14}; % Nx2 vector of torque joint limits
len = args{16}; % length of motion

% initial state
q_c(:, 1) = q0;
dq_c(:, 1) = [0; 0; 0];

W = diag(1./(diff(tb, 1, 2).^2)); % weight matrix for optimization
pos_err = []; vel_err = []; % errors
Kp = 10 * eye(2); Kd = eye(2); % gains for controller

for i=1:len
    % current state (already computed)
    qi = q_c(:, i); dqi = dq_c(:, i);
    
    % dynamic variables
    mqi = substitute(M, q, qi);
    jaci = substitute(jac, q, qi);
    djaci = substitute(djac, [q dq], [qi dqi]);
    ci = substitute(c, [q dq], [qi dqi]);
    fki = substitute(fk, q, qi);
    
    % errors
    e = p(:, i) - fki;
    de = dp(:, i) - jaci * dqi;
    pos_err = [pos_err, full(evalf(norm(e)))];
    vel_err = [vel_err, full(evalf(norm(de)))];
    
    % optimal ddq
    minv = inv(mqi);
    sub_A1 = ((mqi' * W * mqi) \ jaci'); % inv(M' * W * M) * J'
    A = (sub_A1 / (jaci * sub_A1)); % A(qi)
    B = (A * jaci - eye(3)) * minv * (ci - sum(tb, 2));
    
    controller = ddp(:, i) + Kd * de + Kp * e - djaci * dqi;
    ddqi = full(evalf( A * controller + B ));
    
    ddq_c(:, i) = ddqi;
    
    % min norm torque
    trq_c(:, i) = full(evalf(mqi * ddqi + ci));
    
    % next state (using euler integration)
    if i ~= len
        q_c(:, i + 1) = qi + dqi * ts + 0.5 * ddqi * ts^2;
        dq_c(:, i + 1) = dqi + ddqi * ts;
    end
end

outputs = {q_c, dq_c, ddq_c, trq_c, pos_err, vel_err};

end