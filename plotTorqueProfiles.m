function plotTorqueProfiles(t, T, tb)
figure('Name', 'Torque profile');
axis square;
xlim = [0 t(end)]; options = struct('Color', 'red', 'LineWidth', 1);

% first joint
sp(1) = subplot(3,1,1); plot(t, T(1, :)); grid on;
line(xlim, tb(1, 1).*[1, 1], options); line(xlim, tb(1, 2).*[1, 1], options);
ylabel(sp(1), 'Joint 1 [N.m]');
legend('Joint Torque', 'Torque Bound')

% second joint
sp(2) = subplot(3,1,2); plot(t, T(2, :)); grid on;
line(xlim, tb(2, 1).*[1, 1], options); line(xlim, tb(2, 2).*[1, 1], options);
ylabel(sp(2), 'Joint 2 [N.m]');
legend('Joint Torque', 'Torque Bound')

% third joint
sp(3) = subplot(3,1,3); plot(t, T(3, :)); grid on;
line(xlim, tb(3, 1).*[1, 1], options); line(xlim, tb(3, 2).*[1, 1], options);
ylabel(sp(3), 'Joint 3 [N.m]');
legend('Joint Torque', 'Torque Bound')

linkaxes(sp,'x');
title(sp(1), 'Torque profiles');
xlabel(sp(3), 'Time [s]');
end