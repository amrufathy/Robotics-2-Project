function [t, sigma, dsigma, ddsigma, len] = generateTimingLaw(A, L)
% generate motion profiles for specified path length (L) and acceleration
%   bound (A) with bang-bang acceleration profile
    
    % no coast profile (bang-bang)
    V = sqrt(L * A);
    Ts = V/A;
    Tf = ((L * A) + V^2) / (A * V);
    
    ts = 0.001; % sampling time
    len = ceil(Tf / ts); % original motion length (without extension)
    k = 1.5; % extension factor
    t = 0:ts:k*Tf;
    
    % timing law    
     
    % t <= Ts
    idx = (t <= Ts);
    sigma(idx) = 0.5 * A * t(idx).^2;
    dsigma(idx) = A * t(idx);
    ddsigma(idx) = A;
    
    % Ts < t <= Tf
    idx = (Ts < t & t <= Tf);
    sigma(idx) = -0.5 * A * (t(idx) - Tf).^2 + V * Tf - V^2 / A;
    dsigma(idx) = -A * (t(idx) - Tf);
    ddsigma(idx) = -A;
    
    % t > Tf
    idx = (t > Tf);
    sigma(idx) = -0.5 * A * (t(len) - Tf).^2 + V * Tf - V^2 / A;
    dsigma(idx) = 0;
    ddsigma(idx) = 0;
end