% Dynamic model of 3R planar robot using lagrangian method
% Author: Amr Aly
% October 25, 2019

clear; close all; clc;

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

% d(jac)/dt
djac = 0;
for i=1:length(q)
    djac = djac + simplify(diff(jac, q(i)) * dq(i));
end
djac = collect(simplify(djac), ...
    [-1 l cos(q1+q2) cos(q1+q2+q3) sin(q1+q2) sin(q1+q2+q3)]);

% position of CoM
pc = cell(1, N);
pc{1} = [l/2 * cos(q1); l/2 * sin(q1)];
pc{2} = [l * cos(q1) + l/2 * cos(q1 + q2); l * sin(q1) + l/2 * sin(q1 + q2)];
pc{3} = [l * (cos(q1) + cos(q1 + q2)) + l/2 * cos(q1 + q2 + q3);
        l * (sin(q1) + sin(q1 + q2)) + l/2 * sin(q1 + q2 + q3)];

% linear velocities
vc = cell(1, N);
for i=1:N
    vc{i} = 0;
    for j=1:i
        vc{i} = simplify(vc{i} + diff(pc{i}, q(j)) *  dq(j));
    end
end
    
% angular velocities
w = cell(1, N);
w{1} = [0; 0; dq1];
w{2} = [0; 0; dq1 + dq2];
w{3} = [0; 0; dq1 + dq2 + dq3];

% total kinetic energy
Ti = cell(1, N);
for i=1:N
    Ti{i} = simplify(0.5 * m * vc{i}' * vc{i} + ...
        0.5 * Il * w{i}' * w{i});
end
T = simplify(sum([Ti{:}]));

% inertia matrix
M = simplify(hessian(T, dq));
M = collect(M, [m l^2]);
Minv = simplify(pinv(M));

% coriolis and centrifugal terms
c = []; S = [];
for i=1:N
    C = 0.5 * (jacobian(M(:, i), q) + jacobian(M(:, i), q)' - diff(M, q(i)));
    c = [c; simplify(dq' * C * dq)];
    S = [S; simplify(dq' * C)];
end

% substitution of dynamic variables
old = [m l Il]; new = [10 1 10/12];
fk = subs(fk, old, new);
jac = subs(jac, old, new);
jacinv = subs(jacinv, old, new);
M = subs(M, old, new);
Minv = subs(Minv, old, new);
c = subs(c, old, new);
S = subs(S, old, new);

% clear unneeded vars
clear pc vc w Ti T C m l Il q1 q2 q3 dq1 dq2 dq3 i j N old new