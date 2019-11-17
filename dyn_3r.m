% Dynamic model of 3R planar robot using lagrangian method
% Author: Amr Aly
% October 25, 2019

% clear; close all; clc;

import casadi.*;

q1 = MX.sym('q1'); q2 = MX.sym('q2'); q3 = MX.sym('q3'); 
dq1 = MX.sym('dq1'); dq2 = MX.sym('dq2'); dq3 = MX.sym('dq3'); 

% substitution of dynamic variables
m = 1; l = 1; Il = 1/12;

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

% inertia matrix
% precomputed symbolically using matlab
M = [ 
[         cos(q2 + q3) + 3*cos(q2) + cos(q3) + 4, cos(q2 + q3)/2 + (3*cos(q2))/2 + cos(q3) + 5/3, cos(q2 + q3)/2 + cos(q3)/2 + 1/3];
[ cos(q2 + q3)/2 + (3*cos(q2))/2 + cos(q3) + 5/3,                                  cos(q3) + 5/3,                  cos(q3)/2 + 1/3];
[               cos(q2 + q3)/2 + cos(q3)/2 + 1/3,                                cos(q3)/2 + 1/3,                              1/3];
];

Minv = simplify(pinv(M));

% coriolis and centrifugal terms
% precomputed symbolically using matlab
c = [
 - (3*dq2^2*sin(q2))/2 - (dq3^2*sin(q3))/2 - (dq2^2*sin(q2 + q3))/2 - (dq3^2*sin(q2 + q3))/2 - 3*dq1*dq2*sin(q2) - dq1*dq3*sin(q3) - dq2*dq3*sin(q3) - dq1*dq2*sin(q2 + q3) - dq1*dq3*sin(q2 + q3) - dq2*dq3*sin(q2 + q3);
                                                                                                                     (3*dq1^2*sin(q2))/2 - (dq3^2*sin(q3))/2 + (dq1^2*sin(q2 + q3))/2 - dq1*dq3*sin(q3) - dq2*dq3*sin(q3);
                                                                                                                                         (dq1^2*sin(q3))/2 + (dq2^2*sin(q3))/2 + (dq1^2*sin(q2 + q3))/2 + dq1*dq2*sin(q3)
];

% clear unneeded vars
clear fk pc vc w Ti T C m l Il
