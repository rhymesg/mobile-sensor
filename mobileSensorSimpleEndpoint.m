function [ output ] = mobileSensorSimpleEndpoint( input )
% Single-target terminal objective; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.

OC = input.auxdata.OC;
RC = input.auxdata.RC;
tm = input.auxdata.tm;
tf{1} = input.phase(1).finaltime;
x0{2} = input.phase(2).initialstate;
t0{2} = input.phase(2).initialtime;
tf{2} = input.phase(2).finaltime;
xf{2} = input.phase(2).finalstate;
J2 = xf{2}(:,6:14);

xf{1} = input.phase(1).finalstate;
X1 = xf{1}(3:4);
dist = sqrt((X1(1) - OC(1))^2 + (X1(2) - OC(2))^2);
output.eventgroup(1).event = [(x0{2} - xf{1}) (t0{2} - tf{1}) (dist-RC)];
output.eventgroup(2).event = (tf{2} - t0{2} - tm);

J = zeros(3,3);
J(1,1) = J2(1);
J(1,2) = J2(2);
J(1,3) = J2(3);
J(2,1) = J2(4);
J(2,2) = J2(5);
J(2,3) = J2(6);
J(3,1) = J2(7);
J(3,2) = J2(8);
J(3,3) = J2(9);

output.objective = -log2(det(J));

end

