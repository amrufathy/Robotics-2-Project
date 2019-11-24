function [q_c, dq_c, ddq_c, trq_c] = MTN(varargin)
% Min Torque Norm method
disp('MTN method')

t = varargin{1}; % time vector
q0 = varargin{2}; % initial configuration
jac = varargin{3}; % task jacobian
q = varargin{4}; % symbolic q vector
dq = varargin{5}; % symbolic dq vector
p = varargin{6}; % desired cartesian trajectory
dp = varargin{7}; % desired cartesian velocity
ddp = varargin{8}; % desired cartesian acceleration
ts = varargin{9}; % sampling timestep
M = varargin{10}; % inertia matrix of robot
c = varargin{11}; % coriolois & centrifugal terms of robot
djac = varargin{12}; % d(jac)/dt
fk = varargin{13}; % forward kinematics

q_c(size(q0, 2), size(t, 2)) = 0;
dq_c(size(q0, 2), size(t, 2)) = 0;
ddq_c(size(q0, 2), size(t, 2)) = 0;
trq_c(size(q0, 2), size(t, 2)) = 0;

q_c(:, 1) = q0;
dq_c(:, 1) = [0; 0; 0];

for i=1:size(t, 2)
    % current state (already computed)
    qi = q_c(:, i); dqi = dq_c(:, i);
    
    % dynamic variables
    Mqi = substitute(M, q, qi);
    jaci = substitute(jac, q, qi);
    djaci = substitute(djac, [q dq], [qi dqi]);
    ci = substitute(c, [q dq], [qi dqi]);
    fki = substitute(fk, q, qi);
    
    % optimal ddq
    w_jinv = inv(Mqi) * jaci.' * inv(jaci * inv(Mqi) * jaci.');
    controller = ddp(:, i) + Kd * de + Kp * e - djaci * dqi;
    ddqi = full(evalf(...
        w_jinv * controller - (eye() - w_jinv * jaci) * inv(Mqi) * ci ...
    ));

    ddq_c(:, i) = ddqi;
    
    % min norm torque
    t_opt = full(evalf(Mqi * ddqi + ci));
    trq_c(:, i) = t_opt;
    
    % next state (using euler integration)
    q_c(:, i + 1) = qi + dqi * ts + 0.5 * ddqi * ts^2;
    dq_c(:, i + 1) = dqi + ddqi * ts;
    
    break %TODO: remove this
end

end