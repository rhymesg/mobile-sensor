function verify_numerics
% Check callback information and gradient algebra without GPOPS-II.
root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(root);
check_mobile_continuous;
check_mobile_gradient;
fprintf('Mobile sensor numerical checks passed.\n');
end

function check_mobile_continuous
sig = [0.07, 0.08, 0.1];
R = diag(sig.^2);
aux = struct('alt', 20, 'sig_R', sig, 'V', 3, 'OC', [20, 0], ...
    'RC', 30, 'tm', 10, 'L', 3, 'xt_dot', [-1, 0.5, -0.5, 1, -0.1, 0.1], ...
    'multisensor', 1);
eye_row = reshape(eye(3).', 1, []);
targets = [25, -5, 30, -15, 40, 15];

% Single-target callback: phase 2 gains information inside the region.
single = struct('auxdata', aux);
single.phase(1) = struct('time', 0, 'state', [25, -5, 20, 0, 0, eye_row], 'control', 0);
single.phase(2) = struct('time', 1, 'state', [25, -5, 20, 0, 0, eye_row], 'control', 0);
out = mobileSensorSimpleContinuous(single);
assert(all(out(1).dynamics(6:14) == 0), 'Single phase 1 information');
H = measurement_H([25, -5], [20, 0], aux.alt);
expected = reshape((H'*(R\H)).', 1, []);
assert(norm(out(2).dynamics(6:14) - expected, inf) < 1e-10, ...
    'Single phase 2 measurement information');
single.phase(2).state(3:4) = [70, 0];
out = mobileSensorSimpleContinuous(single);
assert(all(out(2).dynamics(6:14) == 0), 'Single outside-region gate');

% Multitarget callback: fixed sensor contributes in both phases; mobile
% sensor adds information only in phase 2 inside the operational circle.
multi = struct('auxdata', aux);
state = [20, 0, 0, targets, repmat(eye_row, 1, aux.L)];
multi.phase(1) = struct('time', 0, 'state', state, 'control', 0);
multi.phase(2) = struct('time', 1, 'state', state, 'control', 0);
out = mobileSensorMultiContinuous(multi);
for k = 1:aux.L
    target = targets(2*k-1:2*k);
    H_fixed = measurement_H(target, [20, -20], aux.alt);
    H_mobile = measurement_H(target, [20, 0], aux.alt);
    J_fixed = H_fixed'*(R\H_fixed)/4;
    J_mobile = H_mobile'*(R\H_mobile);
    cols = 9 + (k-1)*9 + (1:9);
    phase1 = reshape(J_fixed.', 1, []);
    phase2 = reshape((J_fixed + J_mobile).', 1, []);
    assert(norm(out(1).dynamics(cols) - phase1, inf) < 1e-10, ...
        'Multitarget phase 1 information');
    assert(norm(out(2).dynamics(cols) - phase2, inf) < 1e-10, ...
        'Multitarget phase 2 information');
end
multi.phase(2).state(1:2) = [70, 0];
out = mobileSensorMultiContinuous(multi);
for k = 1:aux.L
    target = targets(2*k-1:2*k);
    H_fixed = measurement_H(target, [20, -20], aux.alt);
    J_fixed = H_fixed'*(R\H_fixed)/4;
    cols = 9 + (k-1)*9 + (1:9);
    assert(norm(out(2).dynamics(cols) - reshape(J_fixed.', 1, []), inf) < 1e-10, ...
        'Multitarget outside-region gate');
end
end

function H = measurement_H(target, sensor, altitude)
e = target(1) - sensor(1);
n = target(2) - sensor(2);
u = -altitude;
rho = hypot(e, n);
r2 = rho^2 + u^2;
r = sqrt(r2);
H = [n/rho^2, -e/rho^2, 0; ...
     -e*u/(r2*rho), -n*u/(r2*rho), rho/r2; ...
     e/r, n/r, u/r];
end

function check_mobile_gradient
sensor = [2, 3, 5];
target = [0, 0, 0];
R = [0.0049, 0.001, 0.0003; 0.001, 0.0064, 0.0004; 0.0003, 0.0004, 0.01];
J = [4, 0.7; 0.7, 9]; % retain the helper's leading 2-by-2 approximation
[gx, gy] = control_grad2d(sensor, target, R, J);
h = 1e-5;
dx = [h, 0, 0];
dy = [0, h, 0];
expected = [(rate_cost(sensor+dx, target, R, J) - ...
             rate_cost(sensor-dx, target, R, J))/(2*h), ...
            (rate_cost(sensor+dy, target, R, J) - ...
             rate_cost(sensor-dy, target, R, J))/(2*h)];
actual = [gx, gy];
assert(norm(actual-expected, inf) < 1e-5*max(1, norm(expected, inf)), ...
    'Mobile information-rate gradient finite difference');
end

function cost = rate_cost(sensor, target, R, J)
x = sensor(1)-target(1);
y = sensor(2)-target(2);
z = sensor(3)-target(3);
rho = hypot(x, y);
r2 = rho^2+z^2;
r = sqrt(r2);
H = [y/rho^2, -x/rho^2; ...
     -x*z/(r2*rho), -y*z/(r2*rho); ...
     x/r, y/r];
cost = -0.5*trace(J\(H'*(R\H)));
end
