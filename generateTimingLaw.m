function [t, sigma, dsigma, ddsigma] = generateTimingLaw(A, L)
% generate motion profiles for specified path length (L) and acceleration
%   bound (A) with bang-bang acceleration profile
    
    % no coast profile (bang-bang)
    V = sqrt(L * A);
    Ts = V/A;
    Tf = ((L * A) + V^2) / (A * V);
    
    ts = 0.001; % sampling time
    t = 0:ts:Tf;
    
    % timing law
    sigma(size(t)) = 0;
    dsigma(size(t)) = 0;
    ddsigma(size(t)) = 0;
    
    idx = (t <= Ts);
    
    % t <= Ts
    sigma(idx) = 0.5 * A * t(idx).^2;
    dsigma(idx) = A * t(idx);
    ddsigma(idx) = A;
    
    % t > Ts
    sigma(~idx) = -0.5 * A * (t(~idx) - Tf).^2 + V * Tf - V^2 / A;
    dsigma(~idx) = -A * (t(~idx) - Tf);
    ddsigma(~idx) = -A;
end