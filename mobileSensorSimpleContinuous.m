function [ phaseout ] = mobileSensorSimpleContinuous( input )
% Single-target dynamics and information gain; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.

alt = input.auxdata.alt;
sig_R = input.auxdata.sig_R;
V = input.auxdata.V;
OC = input.auxdata.OC;
RC = input.auxdata.RC;
tm = input.auxdata.tm;

%% phase 1 (approach)

t1 = input.phase(1).time;
x1 = input.phase(1).state;
u1 = input.phase(1).control;
numcol = length(t1);

the = x1(:,5);
XT1_dot = [-2*sin(pi/2/tm*t1) zeros(numcol,1)];
X1_dot = [V*cos(the) V*sin(the)];
the_dot = u1;

J_dot = zeros(numcol,9);

phaseout(1).dynamics = [XT1_dot, X1_dot, the_dot, J_dot];

%% phase 2 (operational area)

t2 = input.phase(2).time;
x2 = input.phase(2).state;
u2 = input.phase(2).control;
numcol = length(t2);

XT2 = x2(:,1:2);
X2 = x2(:,3:4);

the = x2(:,5);
XT2_dot = [-2*sin(pi/2/tm*t2) zeros(numcol,1)];
X2_dot = [V*cos(the) V*sin(the)];
the_dot = u2;

J_dot = zeros(numcol,9);
for k = 1:1:numcol
    
    dist = norm([X2(k,1) - OC(1), X2(k,2) - OC(2)]);
    if (dist < RC)
        e = XT2(k,1) - X2(k,1);
        n = XT2(k,2) - X2(k,2);
        u = 0 - alt;

        H = [n/(e^2+n^2), -e/(e^2+n^2), 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), sqrt(e^2+n^2)/(e^2+n^2+u^2);
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), u/sqrt(e^2+n^2+u^2)];

        R = diag([sig_R(1)^2 sig_R(2)^2 sig_R(3)^2]);
        J = H'*inv(R)*H;

        J_dot(k,1) = J(1,1);
        J_dot(k,2) = J(1,2);
        J_dot(k,3) = J(1,3);
        J_dot(k,4) = J(2,1);
        J_dot(k,5) = J(2,2);
        J_dot(k,6) = J(2,3);
        J_dot(k,7) = J(3,1);
        J_dot(k,8) = J(3,2);
        J_dot(k,9) = J(3,3);
    end
end


phaseout(2).dynamics = [XT2_dot, X2_dot, the_dot, J_dot];

end

