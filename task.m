clear;
close all; clc;
addpath('../casadi-linux-matlabR2014b-v3.5.1')

% invoke 3R dynamic model
dyn_3r;

% acceleration bound
A = 1.4142;

% linear path end points
p0 = [1.4142; -0.4142]; pf = p0 + 0.2; % short path
% p0 = [1.4142; -0.4142]; pf = p0 + 0.83; % long path

% circle center and arc origin (point on the arc)
% cc = [0; 0]; p0r = [1; 0];

% desired trajectory
[t, p, dp, ddp] = generateLinearTrajectory(A, p0, pf);

% check
% plotMotionProfiles(t, p, dp, ddp);

% apply PTR method
tb = [-54 54; -24 24; -6 6];
% q0 = [-45 135 -135]; % degrees
q0 = [-pi/4 3*pi/4 -3*pi/4]; % rad
ts = 0.001;

[q_c, dq_c, ddq_c, trq_c, e, de] = PTR(t, q0, jac, q, dq, p, dp, ddp, ts, M, c, djac, fk, tb);

% [q_c, dq_c, ddq_c, trq_c, e, de] = MTN(t, q0, jac, q, dq, p, dp, ddp, ts, M, c, djac, fk);

% save('MTN_linear_short.mat', 'q_c', 'dq_c', 'ddq_c', 'torque_c')

plotTorqueProfiles(t, trq_c)
plotArm(q_c, p0, pf)
plotErrorProfiles(t, e, de)
