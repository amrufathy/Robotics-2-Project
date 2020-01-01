function [outputs, trq_nxt] = PTR(args)
% Peak Torque Reduction method
disp('PTR method')

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
lb = tb(:, 1); ub = tb(:, 2); % lower and upper torque bounds
pos_err = []; vel_err = []; % errors
violation_count = 0;

for i=1:len - 1
    % current state (already computed)
    qi = q_c(:, i); dqi = dq_c(:, i);
    
    % errors (at current state)
    fki = substitute(fk, q, qi);
    jaci = substitute(jac, q, qi);
    e = p(:, i) - fki;
    de = dp(:, i) - jaci * dqi;
    pos_err = [pos_err, full(evalf(norm(e)))];
    vel_err = [vel_err, full(evalf(norm(de)))];
    
    % next configuration
    qi1 = qi + dqi * ts; % q_i+1
    q_c(:, i + 1) = qi1;
    
    % --------------------------------------------------------------
    % optimize Q to get optimal torque
    import casadi.*;
    
    opti = casadi.Opti();
    Ti = opti.variable(3); % current torque
    
    % dynamic variables
    % current state
    mqi = substitute(M, q, qi); % M(qi)
    cqi = substitute(c, [q dq], [qi dqi]); % c(qi, dqi)
    dqi1 = dqi + inv(mqi) * (Ti - cqi) * ts; % dq_i+1
    
    % next state
    mqi1 = substitute(M, q, qi1); % M(q_i+1)
    cqi1 = substitute(c, [q dq], [qi1 dqi1]); % c(q_i+1, dq_i+1)
    jqi1 = substitute(jac, q, qi1); % J(q_i+1)
    djacqi1 = substitute(djac, [q dq], [qi1 dqi1]); % dJ(q_i+1, dq_i+1)
    ddx = ddp(:, i + 1);
    
    % equations
    sub_A1 = ((mqi1' * W * mqi1) \ jqi1'); % inv(M' * W * M) * J'
    A = (sub_A1 / (jqi1 * sub_A1)); % A(q_i+1)
    
    % objective function (Q) and constraint
    next_torque = mqi1 * A * (jqi1 * inv(mqi1) * cqi1 - djacqi1 * dqi1) + ...
        mqi1 * A * ddx + mqi1 * A * jqi1 * inv(mqi1) * (0 - sum(tb, 2)/2) + sum(tb, 2)/2;
    
    next_bounded_torque = next_torque - sum(tb, 2)/2;
    Q = simplify(0.5 * next_bounded_torque.' * W * next_bounded_torque);
    
    constraint = simplify(jqi1 * dqi1 - dp(:, i + 1));
    
    % perform minimization    
    opti.minimize( Q );
    opti.subject_to( constraint == 0 );
    opti.subject_to( lb <= Ti <= ub );
    
    opts = struct;
    opts.ipopt.print_level = 0;
    opts.print_time = 0;
    opti.solver('ipopt', opts);
    
    % initial torque
    djacqi = substitute(djac, [q dq], [qi dqi]); % dJ(q, dq)
    ddq_init = pinv(jaci) * (ddp(:, i) - djacqi * dqi);
    torq_init = full(evalf(mqi * ddq_init + cqi));
    
    try
        opti.set_initial(Ti, torq_init);
        sol = opti.solve();
        t_opt = sol.value(Ti);
    catch e
        %opti.debug.show_infeasibilities()
        t_opt = opti.debug.value(Ti);
        violation_count = violation_count + 1;
    end
    
    trq_c(:, i) = t_opt;
    trq_nxt(:, i) = full(evalf(substitute(next_torque, Ti, t_opt)));
    % -------------------------------------------------------------------
    
    % compute optimal acceleration generated by torque
    ddq_c(:, i) = full(evalf(inv(mqi) * (t_opt - cqi)));
    
    % compute optimal velocity at next time step
    if i ~= len
        dq_c(:, i + 1) = dqi + ddq_c(:, i) * ts;
    end
end

trq_c(:, len) = trq_c(:, len-1);
trq_nxt(:, len) = trq_nxt(:, len-1);

violation_count

outputs = {q_c, dq_c, ddq_c, trq_c, pos_err, vel_err};

end
