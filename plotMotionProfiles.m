function [] = plotMotionProfiles(t, p, dp, ddp)
    figure;
    sp(1)=subplot(3,1,1);plot(t, p); grid on;
    sp(2)=subplot(3,1,2);plot(t, dp); grid on;
    sp(3)=subplot(3,1,3);plot(t, ddp); grid on;
    linkaxes(sp,'x');
    ylabel(sp(1),'Position [m]');
    ylabel(sp(2),'Velocity [m/s]');
    ylabel(sp(3),'Acceleration [m/s^2]');
    xlabel(sp(3),'Time [s]');
end