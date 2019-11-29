function [t, p, dp, ddp] = generateLinearTrajectory(A, L, p0)
% generate a linear trajectory with L along x/y axis
%   respecting the max acceleration bound (A)
    
    % timing law
    [t, sigma, dsigma, ddsigma] = generateTimingLaw(A, L);
    
    % s = sigma / L; [0, 1]
    
    % trajectory
    p = p0 + sigma .* [1; 1];
    dp = dsigma .* [1; 1]; % dsigma in both directions
    ddp = ddsigma .* [1; 1]; % ddsigma in both directions
end