function [] = plotErrorProfiles(t, e, de)
figure('Name', 'Error Profiles');
axis square;
sp(1) = subplot(2,1,1); plot(t, e); grid on;
sp(2) = subplot(2,1,2); plot(t, de); grid on;
linkaxes(sp,'x');
title(sp(1), 'Error profiles');
ylabel(sp(1), 'pos error [m]');
ylabel(sp(2), 'vel error [m/s]');
xlabel(sp(2), 'Time [s]');
end