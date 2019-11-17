function [t, p, dp, ddp] = generateLinearTrajectory(A, p0, pf)
% generate a linear trajectory from p0 to pf
%   respecting the max acceleration bound (A)
    
    L = norm(pf - pi); % path length
    
    % timing law
    [t, sigma, dsigma, ddsigma] = generateTimingLaw(A, L);
    
    s = sigma / L; % s [0, 1]
    
    % trajectory (note that ||v/L|| = 1)
    p = p0 + s .* (pf - p0);
    dp = dsigma .* [1; 1]; % dsigma in both directions
    ddp = ddsigma .* [1; 1]; % ddsigma in both directions
    
    % rest-to-rest motion
    dp(:, 1) = 0; dp(:, end) = 0;
    ddp(:, 1) = 0; ddp(:, end) = 0;
end