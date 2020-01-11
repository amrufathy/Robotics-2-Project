function plotNorms(t, torques, velocities, move_str)
% plot torque and joint velocities norms

% torque norm
[ptr_trq, mtnb_trq, mtn_trq, mbp_trq] = torques{:};

% column-wise norm
ptr_trq_nrm = vecnorm(ptr_trq, 2, 1);
mtnb_trq_nrm = vecnorm(mtnb_trq, 2, 1);
mtn_trq_nrm = vecnorm(mtn_trq, 2, 1);
mbp_trq_nrm = vecnorm(mbp_trq, 2, 1);

fig = figure('Name', 'Norms');
sp(1) = subplot(2,1,1);
plot(t, mtn_trq_nrm, '-c', 'LineWidth', 3); hold on;
plot(t, ptr_trq_nrm, '-.', ...
    t, mtnb_trq_nrm, '--g', ...
    t, mbp_trq_nrm, ':m', 'LineWidth', 2);
grid on;
title(sp(1), 'Joint torques norm');
ylabel(sp(1), '[N.m]');
% enforce limits for long trajectory
if strcmp(move_str, 'long'), ylim([0 100]); end
legend('MTN', 'PTR', 'MTNB', 'MBP', ...
    'Location', 'northoutside', 'Orientation', 'horizontal');

% velocities norm
[ptr_vel, mtnb_vel, mtn_vel, mbp_vel] = velocities{:};

% column-wise norm
ptr_vel_nrm = vecnorm(ptr_vel, 2, 1);
mtnb_vel_nrm = vecnorm(mtnb_vel, 2, 1);
mtn_vel_nrm = vecnorm(mtn_vel, 2, 1);
mbp_vel_nrm = vecnorm(mbp_vel, 2, 1);

sp(2) = subplot(2,1,2);
plot(t, mtn_vel_nrm, '-c', 'LineWidth', 3); hold on;
plot(t, ptr_vel_nrm, '-.', ...
    t, mtnb_vel_nrm, '--g', ...
    t, mbp_vel_nrm, ':m', 'LineWidth', 2);
grid on;
title(sp(2), 'Joint velocities norm');
ylabel(sp(2), '[rad/s]');
% enforce limits for long trajectory
if strcmp(move_str, 'long'), ylim([0 6]); end

linkaxes(sp,'x');
xlabel(sp(2), 'Time [s]');

% save figure and PNG
print(fig, sprintf('results/%s_norms.png', move_str),'-dpng','-r300');
savefig(sprintf('results/%s_norms.fig', move_str));
end