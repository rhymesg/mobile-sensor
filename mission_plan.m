% Waypoint geometry example; DOI: 10.1016/j.inffus.2018.01.010, Section 3.3.
% See docs/algorithm.md, docs/running.md, and README.md#citation.
clc;
clear;

v = 3;
tm = 10;
ts = 15;
Rc = 30;
psidotmax = pi/5;

X0 = [60, -60];
OC = [20, 0];
[R0 theta_0] = Carte2Polar(OC, X0)

theta_s1 = -42.0669*pi/180;
theta_s2 = -39.4037*pi/180;

T_g = 0.0173;
dt_GDC = 0.2;
n = tm / dt_GDC;

rho = v/psidotmax;

Rw = Rc + 4*rho;

theta_H = acos(Rc/Rw);
% theta_H = 30*pi/180;
theta_L = (theta_0 - theta_H);
theta_U = (theta_0 + theta_H);

N = 5;

theta_h = theta_H/(N-1);

theta_1 = theta_L * 180/pi
theta_2 = (theta_L + 2*theta_h)* 180/pi
theta_3 = (theta_L + 4*theta_h)* 180/pi
theta_4 = (theta_L + 6*theta_h)* 180/pi
theta_5 = theta_U* 180/pi

theta_l = (theta_L + 4*theta_h);
theta_u = (theta_L + 6*theta_h);

theta_w = (theta_l+theta_u)/2;

Xw = Polar2Carte(OC, Rw, theta_w)
Xs1 = Polar2Carte(OC, Rc, theta_s1)
Xs2 = Polar2Carte(OC, Rc, theta_s2)

D = sqrt( (Rw - Rc*cos(theta_h))^2 + (Rc*sin(theta_h))^2 );
d = D/rho;
a = atan2( Rc*sin(theta_h), Rw - Rc*cos(theta_h));
b = a + theta_h;

L = Dubins_RSL_length(a, b, d) * rho 

tr = L/v

% 
% k2_ = 64.23;
% k2 = k2_ * theta_H^2
% k1_ = 3.26 * T_g;
% k1 = k1_ * log(2*theta_H)

% for k = 1:1:100;
%     theta_h = k*0.1*pi/180;
%     D = sqrt( (Rw - Rc*cos(theta_h))^2 + (Rc*sin(theta_h))^2 );
%     d = D/rho;
%     a = atan2( Rc*sin(theta_h), Rw - Rc*cos(theta_h));
%     b = a + theta_h;
% 
%     L = Dubins_RSL_length(a, b, d) * rho;
%     
%     res(k) = L - 4*rho;
%     res2(k) = 64.23*theta_h^2;
% end
% figure;
% plot(res); hold on;
% plot(res2, 'r');

