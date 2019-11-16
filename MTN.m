function [q_c, dq_c, ddq_c, torque_c] = MTN(varargin)
% Min Torque Norm method
disp('MTN method')

t = varargin{1}; % time vector
q0 = varargin{2}; % initial configuration
jac = varargin{3}; % task jac
jacinv = varargin{4}; % inverse of task jac
q = varargin{5}; % symbolic q vector
dq = varargin{6}; % symbolic dq vector
p = varargin{7}; % desired cartesian trajectory
dp = varargin{8}; % desired cartesian velocity
ddp = varargin{9}; % desired cartesian acceleration
ts = varargin{10}; % sampling time
M = varargin{11}; % inertia matrix of robot
c = varargin{12}; % centrifugal terms of robot
tb = varargin{13}; % Nx2 vector of torque joint limits
djac = varargin{14}; % d(jac)/dt

q_c(size(q0, 2), size(t, 2)) = 0;
dq_c(size(q0, 2), size(t, 2)) = 0;
ddq_c(size(q0, 2), size(t, 2)) = 0;
torque_c(size(q0, 2), size(t, 2)) = 0;

q_c(:, 1) = q0;
dq_c(:, 1) = [0; 0; 0];

for i=1:size(t, 2)
    % current state (already computed)
    qi = q_c(:, i); dqi = dq_c(:, i);
    
    % optimal ddq
    Mqi = double(subs(M, q, qi));
    jaci = double(subs(jac, q, qi));
    jinv = double(subs(jacinv, q, qi));
    djaci = double(subs(djac, [q dq], [qi dqi]));
    
    ddphi = 0.5 * pinv(Mqi * (eye() - jinv * jaci)) * ...
        (sum(tb, 2) - 2 * Mqi * jinv * (ddp(:, i) - djaci * dqi));
    
    ddqi = jinv * (ddp(:, i) - djaci * dqi) + (eye() - jinv * jaci) * ddphi;
    ddq_c(:, i) = ddqi;
    
    % min norm torque
    ci = subs(c, [q dq], [qi dqi]);
    torquei = Mqi * ddqi + ci;
    torque_c(:, i) = torquei;
    
    % next state (using euler integration)
    q_c(:, i + 1) = qi + dqi * ts + 0.5 * ddqi * ts^2;
    dq_c(:, i + 1) = dqi + ddqi * ts;
    
    break %TODO: remove this
end

end