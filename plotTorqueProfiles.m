function plotTorqueProfiles(t, torques, tb, move_str)
fig = figure('Name', 'Torque profile');
axis square;
xlim = [0 t(end)]; options = struct('Color', 'red', 'LineWidth', 2);

[ptr_trq, mtnb_trq, mtn_trq, mbp_trq] = torques{:};

% first joint
sp(1) = subplot(3,1,1);
% plot torque commands of each method
plot(t, mtn_trq(1, :), '-c', 'LineWidth', 3); hold on;
plot(t, ptr_trq(1, :), '-.', ...
     t, mtnb_trq(1, :), '--g', ...
     t, mbp_trq(1, :), ':m', 'LineWidth', 2);
grid on;
% plot torque limits
line(xlim, tb(1, 1).*[1, 1], options); line(xlim, tb(1, 2).*[1, 1], options);
ylim([tb(1, 1)*1.5 tb(1, 2)*1.5]); ylabel(sp(1), 'Joint 1 [N.m]');
legend('MTN', 'PTR', 'MTNB', 'MBP', 'Limit', ...
    'Location', 'northoutside', 'Orientation', 'horizontal')

% second joint
sp(2) = subplot(3,1,2); 
plot(t, mtn_trq(2, :), '-c', 'LineWidth', 3); hold on;
plot(t, ptr_trq(2, :), '-.', ...
     t, mtnb_trq(2, :), '--g', ...
     t, mbp_trq(2, :), ':m', 'LineWidth', 2);
grid on;
line(xlim, tb(2, 1).*[1, 1], options); line(xlim, tb(2, 2).*[1, 1], options);
ylim([tb(2, 1)*1.5 tb(2, 2)*1.5]); ylabel(sp(2), 'Joint 2 [N.m]');

% third joint
sp(3) = subplot(3,1,3); 
plot(t, mtn_trq(3, :), '-c', 'LineWidth', 3); hold on;
plot(t, ptr_trq(3, :), '-.', ...
     t, mtnb_trq(3, :), '--g', ...
     t, mbp_trq(3, :), ':m', 'LineWidth', 2);
grid on;
line(xlim, tb(3, 1).*[1, 1], options); line(xlim, tb(3, 2).*[1, 1], options);
ylim([tb(3, 1)*1.5 tb(3, 2)*1.5]); ylabel(sp(3), 'Joint 3 [N.m]');

linkaxes(sp,'x');
xlabel(sp(3), 'Time [s]');

% save figure and PNG
print(fig, sprintf('%s_trq_prf.png', move_str),'-dpng','-r300');
savefig(sprintf('%s_trq_prf.fig', move_str));
end