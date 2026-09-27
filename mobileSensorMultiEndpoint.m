function [ output ] = mobileSensorMultiEndpoint( input )
% Multitarget terminal objective; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.

OC = input.auxdata.OC;
RC = input.auxdata.RC;
tm = input.auxdata.tm;
L = input.auxdata.L;
tf{1} = input.phase(1).finaltime;
x0{2} = input.phase(2).initialstate;
t0{2} = input.phase(2).initialtime;
tf{2} = input.phase(2).finaltime;
xf{2} = input.phase(2).finalstate;
J2 = xf{2}(:,3+2*L+1:end);

xf{1} = input.phase(1).finalstate;
X1 = xf{1}(1:2);
dist = sqrt((X1(1) - OC(1))^2 + (X1(2) - OC(2))^2);
output.eventgroup(1).event = [(x0{2} - xf{1}) (t0{2} - tf{1}) (dist-RC)];
output.eventgroup(2).event = (tf{2} - t0{2} - tm);

Jdet = zeros(1,L);
for k = 1:1:L;
    J = zeros(3,3);
    J = vec2mat(J2( k*9-8: k*9) , 3, 3, 'row');
    
    Jdet(k) = det(J);
end
    
output.objective = -0.5*log2(Jdet(1)*Jdet(2)*Jdet(3));

end

