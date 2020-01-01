%------------
% Main file to run experiments
% Set trajectory parameters then choose
%   torque minimization method
%------------

% clear;
close all; clc;
addpath('../casadi-linux-matlabR2014b-v3.5.1')

% create results dir
if ~exist('results', 'dir')
    mkdir('results')
end

%% invoke 3R dynamic model
dyn_3r;

%% cartesian task
% acceleration bound & path length
A = 2; L = 0.2; move_str = 'short'; % short path
% A = 2; L = 0.5; move_str = 'medium'; % medium path
% A = 1; L = 0.83; move_str = 'long'; % long path

% path endpoints
p0 = [1.4142; -0.4142]; pf = p0 + L;

% circle center and arc origin (point on the arc)
% cc = [0; 0]; p0r = [1; 0];

% desired trajectory
[t, p, dp, ddp, len] = generateLinearTrajectory(A, L, p0);

% check
% plotMotionProfiles(t, p, dp, ddp);

%% torque minimization
% global constants
tb = [-54 54; -24 24; -6 6];
q0 = [-45 135 -135] .* pi/180; % rad
ts = 0.001;

% general input vector
inputs = {t, q0, jac, q, dq, p, dp, ddp, ts, M, c, djac, fk, tb, S, len};

% apply methods
[ptr_out, ptr_nxt_trq] = PTR(inputs);
mtnb_out = MTNB(inputs);
mtn_out = MTN(inputs);
[mbp_out, mbp_nxt_trq] = MBP(inputs);

% outputs = {ptr_out, mtnb_out, mtn_out, mbp_out};
% save(sprintf('results/%s.mat', move_str), 'outputs')

% load(sprintf('results/%s.mat', move_str))
% [ptr_out, mtnb_out, mtn_out, mbp_out] = outputs{:};

%% generate plots
torques = {ptr_out{4}, mtnb_out{4}, mtn_out{4}, mbp_out{4}};
plotTorqueProfiles(t(1:len), torques, tb, move_str)

arm_motions = {ptr_out{1}, mtnb_out{1}, mtn_out{1}, mbp_out{1}};
plotArm(arm_motions, p0, pf, move_str)

velocities = {ptr_out{2}, mtnb_out{2}, mtn_out{2}, mbp_out{2}};
plotNorms(t(1:len), torques, velocities, move_str);

plotEstimatedTorque(t(1:len), ptr_nxt_trq, mbp_nxt_trq, move_str);
