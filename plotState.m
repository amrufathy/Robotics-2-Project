function [] = plotState(t, q, dq, ddq)
% plot state evolution of a robot

% plot q
figure('Name', 'Joint configuration profile');
axis square;
q1 = q(1, :); q2 = q(2, :); q3 = q(3, :); 
sp(1) = subplot(3,1,1); plot(t, q1); grid on;
sp(2) = subplot(3,1,2); plot(t, q2); grid on;
sp(3) = subplot(3,1,3); plot(t, q3); grid on;
linkaxes(sp,'x');
ylabel(sp(1), 'Joint 1 [rad]');
ylabel(sp(2), 'Joint 2 [rad]');
ylabel(sp(3), 'Joint 3 [rad]');
xlabel(sp(3), 'Time [s]');

% plot dq
figure('Name', 'Joint velocity profile');
axis square;
dq1 = dq(1, :); dq2 = q(2, :); dq3 = q(3, :); 
sp(1) = subplot(3,1,1); plot(t, dq1); grid on;
sp(2) = subplot(3,1,2); plot(t, dq2); grid on;
sp(3) = subplot(3,1,3); plot(t, dq3); grid on;
linkaxes(sp,'x');
ylabel(sp(1), 'Joint 1 [rad/s]');
ylabel(sp(2), 'Joint 2 [rad/s]');
ylabel(sp(3), 'Joint 3 [rad/s]');
xlabel(sp(3), 'Time [s]');


% plot ddq
figure('Name', 'Joint acceleration profile');
axis square;
ddq1 = ddq(1, :); ddq2 = ddq(2, :); ddq3 = ddq(3, :); 
sp(1) = subplot(3,1,1); plot(t, ddq1); grid on;
sp(2) = subplot(3,1,2); plot(t, ddq2); grid on;
sp(3) = subplot(3,1,3); plot(t, ddq3); grid on;
linkaxes(sp,'x');
ylabel(sp(1), 'Joint 1 [rad/s^2]');
ylabel(sp(2), 'Joint 2 [rad/s^2]');
ylabel(sp(3), 'Joint 3 [rad/s^2]');
xlabel(sp(3), 'Time [s]');

title(sp(1), 'State profile');

end