% Dynamic model of 3R planar robot using lagrangian method
% Author: Amr Aly
% October 25, 2019

% clear; close all; clc;

import casadi.*;

q1 = MX.sym('q1'); q2 = MX.sym('q2'); q3 = MX.sym('q3'); 
dq1 = MX.sym('dq1'); dq2 = MX.sym('dq2'); dq3 = MX.sym('dq3'); 

q = [q1; q2; q3];
dq = [dq1; dq2; dq3];

% forward kinematics
fk = [cos(q1) + cos(q1 + q2) + cos(q1 + q2 + q3);
        sin(q1) + sin(q1 + q2) + sin(q1 + q2 + q3)];

% task jacobian
jac = jacobian(fk, q);

% d(J)/dt
% precomputed symbolically using matlab
djac = [
[ - (dq1 + dq2)*cos(q1 + q2) - (dq1 + dq2 + dq3)*cos(q1 + q2 + q3) - dq1*cos(q1), - (dq1 + dq2)*cos(q1 + q2) - (dq1 + dq2 + dq3)*cos(q1 + q2 + q3), -(dq1 + dq2 + dq3)*cos(q1 + q2 + q3)];
[ - (dq1 + dq2)*sin(q1 + q2) - (dq1 + dq2 + dq3)*sin(q1 + q2 + q3) - dq1*sin(q1), - (dq1 + dq2)*sin(q1 + q2) - (dq1 + dq2 + dq3)*sin(q1 + q2 + q3), -(dq1 + dq2 + dq3)*sin(q1 + q2 + q3)]
];

% inertia matrix
% precomputed symbolically using matlab
M = [ 
[  10*cos(q2 + q3) + 30*cos(q2) + 10*cos(q3) + 40, 5*cos(q2 + q3) + 15*cos(q2) + 10*cos(q3) + 50/3, 5*cos(q2 + q3) + 5*cos(q3) + 10/3];
[ 5*cos(q2 + q3) + 15*cos(q2) + 10*cos(q3) + 50/3,                               10*cos(q3) + 50/3,                  5*cos(q3) + 10/3];
[               5*cos(q2 + q3) + 5*cos(q3) + 10/3,                                5*cos(q3) + 10/3,                              10/3]
];

% coriolis and centrifugal terms
% precomputed symbolically using matlab
c = [
 (- 15*dq2^2 - 30*dq1*dq2)*sin(q2) + (- 10*dq1*dq3 - 10*dq2*dq3 - 5*dq3^2)*sin(q3) + (- 5*dq2^2 - 10*dq2*dq3 - 10*dq1*dq2 - 5*dq3^2 - 10*dq1*dq3)*sin(q2 + q3);
                                                                       15*dq1^2*sin(q2) + (- 10*dq1*dq3 - 10*dq2*dq3 - 5*dq3^2)*sin(q3) + 5*dq1^2*sin(q2 + q3);
                                                                                               (5*dq1^2 + 10*dq1*dq2 + 5*dq2^2)*sin(q3) + 5*dq1^2*sin(q2 + q3)
];

S = [ 
[ (-15*dq2)*sin(q2) + (-5*dq3)*sin(q3) + (- 5*dq2 - 5*dq3)*sin(q2 + q3), (- 15*dq1 - 15*dq2)*sin(q2) + (-5*dq3)*sin(q3) + (- 5*dq1 - 5*dq2 - 5*dq3)*sin(q2 + q3), (- 5*dq1 - 5*dq2 - 5*dq3)*sin(q3) + (- 5*dq1 - 5*dq2 - 5*dq3)*sin(q2 + q3)];
[                15*dq1*sin(q2) + (-5*dq3)*sin(q3) + 5*dq1*sin(q2 + q3),                                                                        (-5*dq3)*sin(q3),                                          (- 5*dq1 - 5*dq2 - 5*dq3)*sin(q3)];
[                          (5*dq1 + 5*dq2)*sin(q3) + 5*dq1*sin(q2 + q3),                                                                 (5*dq1 + 5*dq2)*sin(q3),                                                                          0]
];

disp('Computed dynamic model...')
% clear unneeded vars
clear q1 q2 q3 dq1 dq2 dq3
