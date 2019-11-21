function [q_c, dq_c, ddq_c, torque_c] = MTN(varargin)
% Min Torque Norm method
disp('MTN method')

import casadi.*;

t = varargin{1}; % time vector
q0 = varargin{2}; % initial configuration
jac = varargin{3}; % task jac
q = varargin{4}; % symbolic q vector
dq = varargin{5}; % symbolic dq vector
p = varargin{6}; % desired cartesian trajectory
dp = varargin{7}; % desired cartesian velocity
ddp = varargin{8}; % desired cartesian acceleration
ts = varargin{9}; % sampling time
M = varargin{10}; % inertia matrix of robot
Minv = varargin{11}; % inverse inertia matrix of robot
c = varargin{12}; % centrifugal terms of robot
djac = varargin{13}; % d(jac)/dt
fk = varargin{14}; % forward kinematics

q_c(size(q0, 2), size(t, 2)) = 0;
dq_c(size(q0, 2), size(t, 2)) = 0;
ddq_c(size(q0, 2), size(t, 2)) = 0;
torque_c(size(q0, 2), size(t, 2)) = 0;

q_c(:, 1) = q0;
dq_c(:, 1) = [0; 0; 0];

% use weighted pseudoinverse
% kd = 0.5 * kp (pd controller)

Kp = 10 * eye(2); Kd = eye(2);
pos_err = [];

for i=1:size(t, 2)
    % current state (already computed)
    qi = q_c(:, i); dqi = dq_c(:, i);
    
    % dynamic variables
    Mqi = substitute(M, q, qi);
    Minvqi = substitute(Minv, q, qi);
    jaci = substitute(jac, q, qi);
    djaci = substitute(djac, [q dq], [qi dqi]);
    ci = substitute(c, [q dq], [qi dqi]);
    fki = substitute(fk, q, qi);
    
    % errors
    e = p(:, i) - fki;
    pos_err = [pos_err, full(evalf(norm(e)))];
    de = dp(:, i) - jaci * dqi;
    
    % optimal ddq
    w_jinv = Mqi * jaci.' * inv(jaci * Mqi * jaci.');
    controller = ddp(:, i) + Kd * de + Kp * e - djaci * dqi;
    ddqi = full(evalf(...
        w_jinv * controller - (eye() - w_jinv * jaci) * Minvqi * ci ...
    ));

    ddq_c(:, i) = ddqi;
    
    % min norm torque
    torquei = full(evalf(Mqi * ddqi + ci));
    torque_c(:, i) = torquei;
    
    % next state (using euler integration)
    q_c(:, i + 1) = qi + dqi * ts + 0.5 * ddqi * ts^2;
    dq_c(:, i + 1) = dqi + ddqi * ts;
    
%     break %TODO: remove this
end

figure('Name', 'Positional Error');
plot(t, pos_err);
axis square; grid on;
xlabel('t [s]'); ylabel('error norm [m]');
end