function plotTorqueProfiles(t, torques, tb, move_str)
% plot & compare torque profiles

fig = figure('Name', 'Torque profile');
axis square;
xlim = [0 t(end)]; options = struct('Color', 'red', 'LineWidth', 2);
ymin = @(i) tb(i, 1).*[1 1]; ymax = @(i) tb(i, 2).*[1 1];

[ptr_trq, mtnb_trq, mtn_trq, mbp_trq] = torques{:};

% first joint
sp(1) = subplot(3,1,1);
% plot torque limits
line(xlim, ymin(1), options, 'HandleVisibility','off'); line(xlim, ymax(1), options); hold on;
% plot torque commands of each method
plot(t, mtn_trq(1, :), '-c', 'LineWidth', 3); hold on;
plot(t, ptr_trq(1, :), '-.', ...
     t, mtnb_trq(1, :), '--g', ...
     t, mbp_trq(1, :), ':m', 'LineWidth', 2);
grid on;
ylim([tb(1, 1)*1.5 tb(1, 2)*1.5]); ylabel(sp(1), 'Joint 1 [N.m]');
legend('Limit', 'MTN', 'PTR', 'MTNB', 'MBP',...
    'Location', 'northoutside', 'Orientation', 'horizontal');

% second joint
sp(2) = subplot(3,1,2);
line(xlim, ymin(2), options); line(xlim, ymax(2), options); hold on;
plot(t, mtn_trq(2, :), '-c', 'LineWidth', 3); hold on;
plot(t, ptr_trq(2, :), '-.', ...
     t, mtnb_trq(2, :), '--g', ...
     t, mbp_trq(2, :), ':m', 'LineWidth', 2);
grid on;
ylim([tb(2, 1)*1.5 tb(2, 2)*1.5]); ylabel(sp(2), 'Joint 2 [N.m]');

% third joint
sp(3) = subplot(3,1,3);
line(xlim, ymin(3), options); line(xlim, ymax(3), options); hold on;
plot(t, mtn_trq(3, :), '-c', 'LineWidth', 3); hold on;
plot(t, ptr_trq(3, :), '-.', ...
     t, mtnb_trq(3, :), '--g', ...
     t, mbp_trq(3, :), ':m', 'LineWidth', 2);
grid on;
ylim([tb(3, 1)*1.5 tb(3, 2)*1.5]); ylabel(sp(3), 'Joint 3 [N.m]');

linkaxes(sp,'x');
xlabel(sp(3), 'Time [s]');

% save figure and PNG
print(fig, sprintf('results/%s_trq_prf.png', move_str),'-dpng','-r300');
savefig(sprintf('results/%s_trq_prf.fig', move_str));
end