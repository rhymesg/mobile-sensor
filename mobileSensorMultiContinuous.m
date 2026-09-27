function [ phaseout ] = mobileSensorMultiContinuous( input )
% Multitarget dynamics and information gain; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.

alt = input.auxdata.alt;
sig_R = input.auxdata.sig_R;
V = input.auxdata.V;
OC = input.auxdata.OC;
RC = input.auxdata.RC;
tm = input.auxdata.tm;
L = input.auxdata.L;
xt_dot = input.auxdata.xt_dot;
multisensor = input.auxdata.multisensor;

R = diag([sig_R(1)^2 sig_R(2)^2 sig_R(3)^2]);
invR = inv(R);

Pos = [20, -20]; % position of operating sensor

%% phase 1 (approach)

t1 = input.phase(1).time;
x1 = input.phase(1).state;
u1 = input.phase(1).control;
numcol = length(t1);

the = x1(:,3);

X1_dot = [V*cos(the) V*sin(the)];
the_dot = u1;
  
XT1_dot = [xt_dot(1)*ones(numcol,1), xt_dot(2)*ones(numcol,1),...
            xt_dot(3)*ones(numcol,1), xt_dot(4)*ones(numcol,1),...
            xt_dot(5)*ones(numcol,1), xt_dot(6)*ones(numcol,1)];
 
J_dot = zeros(numcol,9*L);

if (multisensor == 1)
for k = 1:1:numcol 
    for l = 1:1:L;
        XT1 = x1(k, 3 + l*2-1 : 3 + l*2);

        e = XT1(1) - Pos(1);
        n = XT1(2) - Pos(2);
        u = 0 - alt;

        H = [n/(e^2+n^2), -e/(e^2+n^2), 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 1/sqrt(e^2+n^2);
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), u/sqrt(e^2+n^2+u^2)];

        J = H'*invR*H/4;

        J_dot(k, l*9-8 : l*9) = mat2vec(J, 'row');
    end
end
end

phaseout(1).dynamics = [X1_dot, the_dot, XT1_dot, J_dot];

%% phase 2 (operational area)

t2 = input.phase(2).time;
x2 = input.phase(2).state;
u2 = input.phase(2).control;
numcol = length(t2);

X2 = x2(:,1:2);
the = x2(:,3);

X2_dot = [V*cos(the) V*sin(the)];
the_dot = u2;
       
XT2_dot = [xt_dot(1)*ones(numcol,1), xt_dot(2)*ones(numcol,1),...
            xt_dot(3)*ones(numcol,1), xt_dot(4)*ones(numcol,1),...
            xt_dot(5)*ones(numcol,1), xt_dot(6)*ones(numcol,1)];

J_dot = zeros(numcol,9*L);

for k = 1:1:numcol
    
    % operating sensor
    if (multisensor == 1)
    for l = 1:1:L;
            XT2 = x2(k, 3 + l*2-1 : 3 + l*2);

            e = XT2(1) - Pos(1);
            n = XT2(2) - Pos(2);
            u = 0 - alt;

            H = [n/(e^2+n^2), -e/(e^2+n^2), 0;
                  -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 1/sqrt(e^2+n^2);
                  e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), u/sqrt(e^2+n^2+u^2)];

            J = H'*invR*H/4;

            J_dot(k, l*9-8 : l*9) = mat2vec(J, 'row');
    end
    end
    
    dist = sqrt((X2(k,1) - OC(1))^2 + (X2(k,2) - OC(2))^2);
    if (dist < RC)
    
        for l = 1:1:L;
            XT2 = x2(k, 3 + l*2-1 : 3 + l*2);

            e = XT2(1) - X2(k,1);
            n = XT2(2) - X2(k,2);
            u = 0 - alt;

            H = [n/(e^2+n^2), -e/(e^2+n^2), 0;
                  -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 1/sqrt(e^2+n^2);
                  e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), u/sqrt(e^2+n^2+u^2)];

            J = H'*invR*H;

            J_dot(k, l*9-8 : l*9) = J_dot(k, l*9-8 : l*9) + mat2vec(J, 'row');

        end
        
    end
end


phaseout(2).dynamics = [X2_dot, the_dot, XT2_dot, J_dot];

end

