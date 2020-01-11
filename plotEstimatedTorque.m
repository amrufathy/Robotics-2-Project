function plotEstimatedTorque(t, ptr_trq, mbp_trq, move_str)
% plot estimated future torque profiles (PTR & MBP methods only)

ptr_trq_nrm = vecnorm(ptr_trq, 2, 1);
mbp_trq_nrm = vecnorm(mbp_trq, 2, 1);

fig = figure('Name', 'Estimated Torque');
plot(t, ptr_trq_nrm, '-.', ...
    t, mbp_trq_nrm, ':m', 'LineWidth', 2);
grid on;
% title('Estimated torque norm');
ylabel('[N.m]');
xlabel('Time [s]');
legend('PTR', 'MBP (T = 100T_s)', ...
    'Location', 'northoutside', 'Orientation', 'horizontal');

% save figure and PNG
print(fig, sprintf('results/%s_est_trq.png', move_str),'-dpng','-r300');
savefig(sprintf('results/%s_est_trq.fig', move_str));
end