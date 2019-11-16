function [t, p, dp, ddp] = generateCircularTrajectory(A, c, p0)
% generat a circular trajectory centered at (c) with (p0) as origin
%   of the circular arc and respecting the max acceleration bound (A)
    
    r = norm(p0 - c); % radius of circle
    L = 2 * pi * r; % circumference
    
    % timing law
    [t, sigma, dsigma, ddsigma] = generateTimingLaw(A, L);
    
    % trajectory
    s = sigma / r; % arc length [0, 2pi]
    p = c + r * [cos(s); sin(s)];
    dp = dsigma .* [-sin(s); cos(s)];
    ddp = ddsigma .* [-sin(s); cos(s)] - ...
        dsigma.^2/r .* [cos(s); sin(s)];
    
    % rest-to-rest motion
    dp(:, 1) = 0; dp(:, end) = 0;
    ddp(:, 1) = 0; ddp(:, end) = 0;
end