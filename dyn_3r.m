% Dynamic model of 3R planar robot using lagrangian method
% Author: Amr Aly
% October 25, 2019

% clear; close all; clc;

syms q1 q2 q3 dq1 dq2 dq3 real
syms m l Il real

N = 3;

q = [q1; q2; q3];
dq = [dq1; dq2; dq3];

% forward kinematics
fk = l * [cos(q1) + cos(q1 + q2) + cos(q1 + q2 + q3);
        sin(q1) + sin(q1 + q2) + sin(q1 + q2 + q3)];

% task jacobian
jac = jacobian(fk, q);
jacinv = simplify(pinv(jac));
djac = -1 * [
    [cos(q1) * dq1 + cos(q1 + q2) * (dq1 + dq2) + cos(q1 + q2 + q3) * (dq1 + dq2 + dq3), ...
    cos(q1 + q2) * (dq1 + dq2) + cos(q1 + q2 + q3) * (dq1 + dq2 + dq3), ...
    cos(q1 + q2 + q3) * (dq1 + dq2 + dq3)];
    
    [sin(q1) * dq1 + sin(q1 + q2) * (dq1 + dq2) + sin(q1 + q2 + q3) * (dq1 + dq2 + dq3), ...
    sin(q1 + q2) * (dq1 + dq2) + sin(q1 + q2 + q3) * (dq1 + dq2 + dq3), ...
    sin(q1 + q2 + q3) * (dq1 + dq2 + dq3)]
];

% position of CoM
pc = cell(1, N);
pc{1} = [l/2 * cos(q1); l/2 * sin(q1)];
pc{2} = [l * cos(q1) + l/2 * cos(q1 + q2); l * sin(q1) + l/2 * sin(q1 + q2)];
pc{3} = [l * (cos(q1) + cos(q1 + q2)) + l/2 * cos(q1 + q2 + q3);
        l * (sin(q1) + sin(q1 + q2)) + l/2 * sin(q1 + q2 + q3)];

% angular velocities
w = cell(1, N);
w{1} = [0; 0; dq1];
w{2} = [0; 0; dq1 + dq2];
w{3} = [0; 0; dq1 + dq2 + dq3];

vc = cell(1, N);
for i=1:N
    vc{i} = 0;
    for j=1:i
        vc{i} = simplify(vc{i} + diff(pc{i}, q(j)) *  dq(j));
    end
end

% total kinetic energy
Ti = cell(1, N);
for i=1:N
    Ti{i} = simplify(0.5 * m * vc{i}' * vc{i} + ...
        0.5 * Il * w{i}' * w{i});
end
T = simplify(sum([Ti{:}]));

% inertia matrix
for i=1:N
    for j=1:N
        M(i, j) = simplify(diff(diff(T, dq(i)), dq(j)));
    end
end
M = collect(M, [m l^2]);
Minv = simplify(pinv(M));

% coriolis and centrifugal terms
c = [];
for i=1:N
    C = 0.5 * (jacobian(M(:, i), q) + jacobian(M(:, i), q)' - diff(M, q(i)));
    c = [c; simplify(dq' * C * dq)];
end

% substitution of dynamic variables
old = [m l Il]; new = [1 1 1/12];
fk = subs(fk, old, new);
jac = subs(jac, old, new);
jacinv = subs(jacinv, old, new);
M = subs(M, old, new);
Minv = subs(Minv, old, new);
c = subs(c, old, new);

% clear unneeded vars
clear pc vc w Ti T C m l Il
