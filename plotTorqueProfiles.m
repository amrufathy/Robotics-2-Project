function plotTorqueProfiles(t, T)
figure('Name', 'Torque profile');
axis square;
sp(1) = subplot(3,1,1); plot(t, T(1, :)); grid on;
sp(2) = subplot(3,1,2); plot(t, T(2, :)); grid on;
sp(3) = subplot(3,1,3); plot(t, T(3, :)); grid on;
linkaxes(sp,'x');
ylabel(sp(1), 'Joint 1');
ylabel(sp(2), 'Joint 2');
ylabel(sp(3), 'Joint 3');
xlabel(sp(3), 'Time [s]');
end