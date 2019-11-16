function [] = PTR(varargin)
% Peak Torque Reduction method
disp('PTR method')

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
Minv = varargin{12}; % inverse inertia matrix of robot
c = varargin{13}; % centrifugal terms of robot
tb = varargin{14}; % Nx2 vector of torque joint limits
djac = varargin{15}; % d(jac)/dt

q_command(size(q0, 2), size(t, 2)) = 0;
dq_command(size(q0, 2), size(t, 2)) = 0;
ddq_command(size(q0, 2), size(t, 2)) = 0;
torque_command(size(q0, 2), size(t, 2)) = 0;

% initial state
q_command(:, 1) = q0;
dq_command(:, 1) = [0; 0; 0];

Wm = diag(diff(tb, 1, 2)).^2; % weight matrix for optimization

lb = tb(:, 1);
ub = tb(:, 2);

for i=1:size(t, 2)
    % current state (already computed)
    qi = q_command(:, i); dqi = dq_command(:, i);
    
    % next state
    qi1 = qi + dqi * ts; % q_i+1
    q_command(:, i + 1) = qi1;
    
    % known: qi, q_i+1, dqi
    
    % TODO --------------------------------------------------------------
    % optimize Q to get optimal torque
%     import casadi.*;
    
    Ti = sym('Ti'); % current torque
    minv = double(subs(Minv, q, qi)); % Minv(q)
    cqi = double(subs(c, [q dq], [qi dqi])); % c(qi, dqi)
    dqi1 = dqi + vpa(minv * (Ti - cqi)) * ts; % dq_i+1
    
    Mqi1 = double(subs(M, q, qi1)); % M(q_i+1)
    cqi1 = simplify(subs(c, [q dq], [qi1 dqi1])); % c(q_i+1, dq_i+1)
    % for ddqi optimal
    jqi1 = double(subs(jac, q, qi1)); % J(q_i+1)
    sub_A1 = double((Mqi1.' * Wm * Mqi1) \ jqi1.'); % inv(M' * W * M) * J'
    A = double(sub_A1 / (jqi1 * sub_A1)); % A(q_i+1)
    B = simplify((A * jqi1 - eye()) * Minv * (cqi1 - sum(tb, 2)/2));
    B = simplify(subs(B, [q dq], [qi1 dqi1])); % B(q_i+1, dq_i+1)
    % ----------------
    ddx = ddp(:, i + 1);
    djacqi1 = simplify(subs(djac, [q dq], [qi1 dqi1])); % dJ(q_i+1, dq_i+1)
    
    next_torque = simplify(Mqi1 * A * (ddx - djacqi1 * dqi1) + Mqi1 * B + cqi); % T(i+1)
    next_bounded_torque = next_torque - sum(tb, 2)/2;
    Q = simplify(0.5 * next_bounded_torque.' * Wm * next_bounded_torque);
%     Q = matlabFunction(Q);
%     Q([0; 0; 0]);
    
    constraint = jqi1 * vpa(minv * (Ti - cqi)) * ts + vpa(jqi1 * dqi) - ...
        vpa(dp(:, i + 1));
    
    % TODO: make sure Q and the constraint are correct then use casadi

    

    % -------------------------------------------------------------------
    
    % compute optimal acceleration generated by torque
    ddq_command(:, i) = minv * (torque_command(:, i) - cqi); % no gravity
    
    % compute optimal velocity at next time step
    dq_command(:, i + 1) = dqi + ddq_command(:, i) * ts;
    
    break
end

end