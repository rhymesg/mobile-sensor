% Multitarget dispatch experiment; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.
clear;
clc;
close all;

%%
cx = 20; % center of C
cy = 0;
Rc = 30;
tm = 10;
alt = 20;
V = 3;

% grad
psi_dot_max = pi/5;
tau = 0.2;
Pos = [20, -20];
ratio = 4;

% sensor2
Pos2 = [0, -10];
% sensor3
Pos3 = [10, 20];

% load('result.mat', 'output');
load('result_ms2_opt.mat', 'output');
res = output.result;

multisensor = res.setup.auxdata.multisensor;

X = [res.solution.phase(1).state(:,1:2); res.solution.phase(2).state(:,1:2)];

xt_init = [25, -5, ...
          30, -15, ...
          40, 15,];
xt_dot = [-1, 0.5, ...
 -0.5, 1, ...
  -0.1, 0.1];

% compute J_opt
len1 = length(res.solution.phase(1).time);
len2 = length(res.solution.phase(2).time);

tf1 = res.solution.phase(1).time(end);
tf2 = tf1 + tm;

dt1 = res.solution.phase(1).time(end)/len1;
dt2 = tm/len2;

std_prop = [0.2 0.2];
std_meas = [0.07 0.07 0.1]/3;

F1 = [zeros(2,2), eye(2); zeros(2,4)];
F2 = [zeros(2,2), eye(2); zeros(2,4)];
F3 = [zeros(2,2), eye(2); zeros(2,4)];
Q = [zeros(2,4); zeros(2,2), diag(std_prop.*std_prop)*dt1];
R = diag(std_meas.*std_meas)/dt2;
invR = inv(R);
Rm = diag([std_meas.*std_meas, std_meas.*std_meas, std_meas.*std_meas])/dt2;
invRm = inv(Rm);

J1 = [50*eye(2), zeros(2,2); zeros(2,2), 10*eye(2)];
J2 = [100*eye(2), zeros(2,2); zeros(2,2), 50*eye(2)];
J3 = [150*eye(2), zeros(2,2); zeros(2,2), 50*eye(2)];

I1(1) = -0.5*log(det(J1));
I2(1) = -0.5*log(det(J2));
I3(1) = -0.5*log(det(J3));

XT(1,:) = xt_init;
for k = 1:1:len1-1;
    
    J1_dot = -J1*F1' - F1*J1 - J1*Q*J1;
    J2_dot = -J2*F2' - F2*J2 - J2*Q*J2;
    J3_dot = -J3*F3' - F3*J3 - J3*Q*J3;
    
    if (multisensor == 1)
        e = XT(k,1) - Pos(1);
        n = XT(k,2) - Pos(2);
        u = 0 - alt;
        H1_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos2(1);
        n = XT(k,2) - Pos2(2);
        u = 0 - alt;
        H1_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos3(1);
        n = XT(k,2) - Pos3(2);
        u = 0 - alt;
        H1_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H1 = [H1_1; H1_2; H1_3];
        
        e = XT(k,3) - Pos(1);
        n = XT(k,4) - Pos(2);
        u = 0 - alt;
        H2_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos2(1);
        n = XT(k,4) - Pos2(2);
        u = 0 - alt;
        H2_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos3(1);
        n = XT(k,4) - Pos3(2);
        u = 0 - alt;
        H2_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H2 = [H2_1; H2_2; H2_3];
        
        e = XT(k,5) - Pos(1);
        n = XT(k,6) - Pos(2);
        u = 0 - alt;
        H3_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos2(1);
        n = XT(k,6) - Pos2(2);
        u = 0 - alt;
        H3_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos3(1);
        n = XT(k,6) - Pos3(2);
        u = 0 - alt;
        H3_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H3 = [H3_1; H3_2; H3_3];
           
        J1_dot = J1_dot + H1'*invRm/ratio*H1;
        J2_dot = J2_dot + H2'*invRm/ratio*H2;
        J3_dot = J3_dot + H3'*invRm/ratio*H3;

    end
    
    J1 = J1 + J1_dot*dt1;
    J2 = J2 + J2_dot*dt1;
    J3 = J3 + J3_dot*dt1;
    
    I1(k+1) = -0.5*log(det(J1));
    I2(k+1) = -0.5*log(det(J2));
    I3(k+1) = -0.5*log(det(J3));
    
    XT(k+1,:) = XT(k,:) + xt_dot*dt1;
end
Js1 = J1;
Js2 = J2;
Js3 = J3;
Is1 = I1;
Is2 = I2;
Is3 = I3;
I_tot1 = I1 + I2 + I3;

Jp1 = J1;
Jp2 = J2;
Jp3 = J3;
I_tot_p = I1 + I2 + I3;

XT1 = XT;

% plus planning time
tp = 10;
lenp = floor(tp/dt1);
for k = len1:1:len1+lenp;
    
    Jp1_dot = -Jp1*F1' - F1*Jp1 - Jp1*Q*Jp1;
    Jp2_dot = -Jp2*F2' - F2*Jp2 - Jp2*Q*Jp2;
    Jp3_dot = -Jp3*F3' - F3*Jp3 - Jp3*Q*Jp3;
    
    if (multisensor == 1)
        e = XT(len1,1) - Pos(1);
        n = XT(len1,2) - Pos(2);
        u = 0 - alt;
        H1_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(len1,1) - Pos2(1);
        n = XT(len1,2) - Pos2(2);
        u = 0 - alt;
        H1_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(len1,1) - Pos3(1);
        n = XT(len1,2) - Pos3(2);
        u = 0 - alt;
        H1_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H1 = [H1_1; H1_2; H1_3];
        
        e = XT(len1,3) - Pos(1);
        n = XT(len1,4) - Pos(2);
        u = 0 - alt;
        H2_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(len1,3) - Pos2(1);
        n = XT(len1,4) - Pos2(2);
        u = 0 - alt;
        H2_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(len1,3) - Pos3(1);
        n = XT(len1,4) - Pos3(2);
        u = 0 - alt;
        H2_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H2 = [H2_1; H2_2; H2_3];
        
        e = XT(len1,5) - Pos(1);
        n = XT(len1,6) - Pos(2);
        u = 0 - alt;
        H3_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(len1,5) - Pos2(1);
        n = XT(len1,6) - Pos2(2);
        u = 0 - alt;
        H3_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(len1,5) - Pos3(1);
        n = XT(len1,6) - Pos3(2);
        u = 0 - alt;
        H3_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H3 = [H3_1; H3_2; H3_3];
              
              Jp1_dot = Jp1_dot + H1'*invRm/ratio*H1;
              Jp2_dot = Jp2_dot + H2'*invRm/ratio*H2;
              Jp3_dot = Jp3_dot + H3'*invRm/ratio*H3;

    end
    
    
    Jp1 = Jp1 + Jp1_dot*dt1;
    Jp2 = Jp2 + Jp2_dot*dt1;
    Jp3 = Jp3 + Jp3_dot*dt1;
    
    Ip1 = -0.5*log(det(Jp1));
    Ip2 = -0.5*log(det(Jp2));
    Ip3 = -0.5*log(det(Jp3));
    I_tot_p(k+1) = Ip1 + Ip2 + Ip3;
end

% lenp = length(I_tot_p)-len1;
I_tot_ps = I_tot_p;
Jps1 = Jp1;
Jps2 = Jp2;
Jps3 = Jp3;

% operational area
Q = [zeros(2,4); zeros(2,2), diag(std_prop.*std_prop)*dt2];
for k = len1:1:len1+len2-1;
    
    e = XT(k,1) - X(k,1);
    n = XT(k,2) - X(k,2);
    u = 0 - alt;

    H1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
          -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
          e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
      
    e = XT(k,3) - X(k,1);
    n = XT(k,4) - X(k,2);
    u = 0 - alt;

    H2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
          -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
          e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
      
    e = XT(k,5) - X(k,1);
    n = XT(k,6) - X(k,2);
    u = 0 - alt;

    H3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
          -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
          e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
    
    J1_dot = -J1*F1' - F1*J1 - J1*Q*J1 + H1'*invR*H1;
    J2_dot = -J2*F2' - F2*J2 - J2*Q*J2 + H2'*invR*H2;
    J3_dot = -J3*F3' - F3*J3 - J3*Q*J3 + H3'*invR*H3;
    
    Jp1_dot = -Jp1*F1' - F1*Jp1 - Jp1*Q*Jp1 + H1'*invR*H1;
    Jp2_dot = -Jp2*F2' - F2*Jp2 - Jp2*Q*Jp2 + H2'*invR*H2;
    Jp3_dot = -Jp3*F3' - F3*Jp3 - Jp3*Q*Jp3 + H3'*invR*H3;
    

    if (multisensor == 1)
        e = XT(k,1) - Pos(1);
        n = XT(k,2) - Pos(2);
        u = 0 - alt;
        H1_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos2(1);
        n = XT(k,2) - Pos2(2);
        u = 0 - alt;
        H1_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos3(1);
        n = XT(k,2) - Pos3(2);
        u = 0 - alt;
        H1_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H1 = [H1_1; H1_2; H1_3];
        
        e = XT(k,3) - Pos(1);
        n = XT(k,4) - Pos(2);
        u = 0 - alt;
        H2_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos2(1);
        n = XT(k,4) - Pos2(2);
        u = 0 - alt;
        H2_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos3(1);
        n = XT(k,4) - Pos3(2);
        u = 0 - alt;
        H2_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H2 = [H2_1; H2_2; H2_3];
        
        e = XT(k,5) - Pos(1);
        n = XT(k,6) - Pos(2);
        u = 0 - alt;
        H3_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos2(1);
        n = XT(k,6) - Pos2(2);
        u = 0 - alt;
        H3_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos3(1);
        n = XT(k,6) - Pos3(2);
        u = 0 - alt;
        H3_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H3 = [H3_1; H3_2; H3_3];
              
              J1_dot = J1_dot + H1'*invRm/ratio*H1;
              J2_dot = J2_dot + H2'*invRm/ratio*H2;
              J3_dot = J3_dot + H3'*invRm/ratio*H3;
              
              Jp1_dot = Jp1_dot + H1'*invRm/ratio*H1;
              Jp2_dot = Jp2_dot + H2'*invRm/ratio*H2;
              Jp3_dot = Jp3_dot + H3'*invRm/ratio*H3;
        end
    
    J1 = J1 + J1_dot*dt2;
    J2 = J2 + J2_dot*dt2;
    J3 = J3 + J3_dot*dt2;
    
    I1(k+1) = -0.5*log(det(J1));
    I2(k+1) = -0.5*log(det(J2));
    I3(k+1) = -0.5*log(det(J3));
    
    Jp1 = Jp1 + Jp1_dot*dt1;
    Jp2 = Jp2 + Jp2_dot*dt1;
    Jp3 = Jp3 + Jp3_dot*dt1;
    
    Ip1 = -0.5*log(det(Jp1));
    Ip2 = -0.5*log(det(Jp2));
    Ip3 = -0.5*log(det(Jp3));
    I_tot_p(lenp+k+1) = Ip1 + Ip2 + Ip3;
   
    XT(k+1,:) = XT(k,:) + xt_dot*dt2;

end

I_tot = I1 + I2 + I3;
reference = I_tot(end)
% I_tot_p = [I_tot_p, (I_tot(len1+1:end) + (I_tot_p(end) - I_tot1(end))*ones(1,len2))];

%%
% find best starting point
the_L = -108.6470 * pi/180;
the_U = -3.9729 * pi/180;
% the_l = -82.4785 * pi/180;
% the_u = -56.3099 * pi/180; % ss
% the_l = -56.3099 * pi/180;
% the_u = -30.1414 * pi/180; % ms
the_l = -30.1414 * pi/180;
the_u = -3.9729 * pi/180; % ms2

% the_l = the_L;
% the_u = the_U;
num = 100;
N = 5;

len2g = len2;
dt2g = tm/len2g;
Q = [zeros(2,4); zeros(2,2), diag(std_prop.*std_prop)*dt2g];
R = diag(std_meas.*std_meas)/dt2g;
invR = inv(R);

Ig1 = [Is1 zeros(1,len2g)]; 
Ig2 = [Is2 zeros(1,len2g)]; 
Ig3 = [Is3 zeros(1,len2g)]; 
Ig_tot = zeros(1,len1+len2g);
psi = zeros(1,len1+len2g);

Ig_f_best = 0;
bestidx = 0;
for angidx = 1:1:15;
    
    Xg = zeros(len1+len2g,2);
    
    angle = the_l + (the_u - the_l)/num * angidx;
    Xg(len1,1) = cx + Rc*cos(angle);
    Xg(len1,2) = cy + Rc*sin(angle);

    J1 = Js1;
    J2 = Js2;
    J3 = Js3;
    
    psi(len1) = 3/4*pi;%3.5/4*pi;

    for k = len1:1:len1+len2g-1;
        
        e = XT(k,1) - Xg(k,1);
        n = XT(k,2) - Xg(k,2);
        u = 0 - alt;

        H1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
     
        e = XT(k,3) - Xg(k,1);
        n = XT(k,4) - Xg(k,2);
        u = 0 - alt;

        H2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
   
        e = XT(k,5) - Xg(k,1);
        n = XT(k,6) - Xg(k,2);
        u = 0 - alt;

        H3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];

        J1_dot = -J1*F1' - F1*J1 - J1*Q*J1 + H1'*invR*H1;
        J2_dot = -J2*F2' - F2*J2 - J2*Q*J2 + H2'*invR*H2;
        J3_dot = -J3*F3' - F3*J3 - J3*Q*J3 + H3'*invR*H3;
        
       if (multisensor == 1)
          e = XT(k,1) - Pos(1);
        n = XT(k,2) - Pos(2);
        u = 0 - alt;
        H1_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos2(1);
        n = XT(k,2) - Pos2(2);
        u = 0 - alt;
        H1_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos3(1);
        n = XT(k,2) - Pos3(2);
        u = 0 - alt;
        H1_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H1 = [H1_1; H1_2; H1_3];
        
        e = XT(k,3) - Pos(1);
        n = XT(k,4) - Pos(2);
        u = 0 - alt;
        H2_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos2(1);
        n = XT(k,4) - Pos2(2);
        u = 0 - alt;
        H2_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos3(1);
        n = XT(k,4) - Pos3(2);
        u = 0 - alt;
        H2_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H2 = [H2_1; H2_2; H2_3];
        
        e = XT(k,5) - Pos(1);
        n = XT(k,6) - Pos(2);
        u = 0 - alt;
        H3_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos2(1);
        n = XT(k,6) - Pos2(2);
        u = 0 - alt;
        H3_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos3(1);
        n = XT(k,6) - Pos3(2);
        u = 0 - alt;
        H3_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H3 = [H3_1; H3_2; H3_3];
              
              J1_dot = J1_dot + H1'*invRm/ratio*H1;
              J2_dot = J2_dot + H2'*invRm/ratio*H2;
              J3_dot = J3_dot + H3'*invRm/ratio*H3;
        end

        J1 = J1 + J1_dot*dt2g;
        J2 = J2 + J2_dot*dt2g;
        J3 = J3 + J3_dot*dt2g;
        
        Ig1(k+1) = -0.5*log(det(J1));
        Ig2(k+1) = -0.5*log(det(J2));
        Ig3(k+1) = -0.5*log(det(J3));
       
        grad_sum = [0 0];
        grad = control_grad2d([Xg(k,1) Xg(k,2) alt], [XT(k,1) XT(k,2) 0], R, J1);
        grad_sum = grad_sum + grad;
        grad = control_grad2d([Xg(k,1) Xg(k,2) alt], [XT(k,3) XT(k,4) 0], R, J2);
        grad_sum = grad_sum + grad;
        grad = control_grad2d([Xg(k,1) Xg(k,2) alt], [XT(k,5) XT(k,6) 0], R, J3);
        grad_sum = grad_sum + grad;

        grad_sum;
        psi_cmd = -atan2(grad_sum(2), grad_sum(1));   
        
        psi_dot_cmd = (psi_cmd - psi(k))/tau;
        if (psi_dot_cmd > psi_dot_max)
            psi_dot_cmd = psi_dot_max;
        elseif (psi_dot_cmd < -psi_dot_max)
            psi_dot_cmd = -psi_dot_max;
        end
        
        Xg(k+1,:) = Xg(k,:) + [V*cos(psi(k)) V*sin(psi(k))]*dt2g;
        psi(k+1) = psi(k) + psi_dot_cmd*dt2g;
%         pause
    end
    
    Ig_tot = Ig1 + Ig2 + Ig3;  
    
%     if (mod((angidx-1), num/N) == 0)
%         Ig_tot(end)
%     end
        
    if (Ig_tot(end) < Ig_f_best)
        bestidx = angidx;
        Ig_f_best = Ig_tot(end);
        Xg_best = Xg;
        Ig_best_tot = Ig_tot;
        psi_best = psi;
    end
    
end
% 
bestidx
angle = the_l + (the_u - the_l)/num * bestidx * 180/pi

proposed = Ig_f_best(end)

%% plot

time1 = dt1:dt1:tf1;
time = [time1 tf1+dt2:dt2:tf2];
timeg = [time1 tf1+dt2g:dt2g:tf2];

figure(1);
plot(time(len1:len1+len2), I_tot(len1:len1+len2), 'linewidth', 2); hold on;
grid on;

figure(2);
time_p = [-(dt1*lenp:-dt1:dt1)];
plot([time_p time], I_tot_p, 'linewidth', 2); hold on;

reference_tp = I_tot_p(end)

Cb = zeros(100,2);
Cbr = zeros(100,2);
Cbw = zeros(100,2);
Rw = Rc + 4*V/psi_dot_max - 5;
the_w = (the_u + the_l)/2;
a = the_L;
b = the_U;
for i = 1:1:100;
    ang = the_l + (the_u-the_l)*i/100;
    Cbr(i,1) = cx + Rc*cos(ang);
    Cbr(i,2) = cy + Rc*sin(ang);
    Cbw(i,1) = cx + Rw*cos((ang-the_w)/10+the_w);
    Cbw(i,2) = cy + Rw*sin((ang-the_w)/10+the_w);
    ang = a + (b-a)*i/100;
    Cb(i,1) = cx + Rc*cos(ang);
    Cb(i,2) = cy + Rc*sin(ang);
end

figure(3);
 hold on;
plot(XT(:,1), XT(:,2), 'k--', 'linewidth', 1.5);
plot(X(:,1), X(:,2), 'linewidth', 2.5);
plot(Cb(:,1), Cb(:,2), 'k--');
plot(Cbr(:,1), Cbr(:,2), 'k', 'linewidth', 1.5);
plot(Cbw(:,1), Cbw(:,2), 'k', 'linewidth', 1);
plot(XT(:,5), XT(:,6), 'k--', 'linewidth', 1.5);
plot(XT(:,3), XT(:,4), 'k--', 'linewidth', 1.5);

plot(XT(1,1), XT(1,2), 'k^', 'markerfacecolor', [0.9 0.9 0.9]);
plot(XT(1,3), XT(1,4), 'k^', 'markerfacecolor', [0.9 0.9 0.9]);
plot(XT(1,5), XT(1,6), 'k^', 'markerfacecolor', [0.9 0.9 0.9]);
plot(XT(end,1), XT(end,2), 'kd', 'markerfacecolor', [0.9 0.9 0.9]);
plot(XT(end,3), XT(end,4), 'kd', 'markerfacecolor', [0.9 0.9 0.9]);
plot(XT(end,5), XT(end,6), 'kd', 'markerfacecolor', [0.9 0.9 0.9]);
legend('Targets', 'Optimal', 'Gradient', 'C_b', '[\theta_l,\theta_u]');
xlabel('x (m)', 'fontsize', 12);
ylabel('y (m)', 'fontsize', 12);

%% error optimal

load('result_ms2_opt_err.mat', 'output');
res = output.result;

multisensor = res.setup.auxdata.multisensor;

X = [res.solution.phase(2).state(:,1:2)];

% compute J_opt
len1e = length(res.solution.phase(1).time);
len2 = length(res.solution.phase(2).time);

dt2 = tm/len2;

R = diag(std_meas.*std_meas)/dt2;
invR = inv(R);

J1 = Js1;
J2 = Js2;
J3 = Js3;
I1 = Is1;
I2 = Is2;
I3 = Is3;

Jp1 = Jps1;
Jp2 = Jps2;
Jp3 = Jps3;
I_tot_p = I_tot_ps;

XT = XT1;

% operational area
Q = [zeros(2,4); zeros(2,2), diag(std_prop.*std_prop)*dt2];
for k = len1:1:len1+len2-1;
    
    kk = k-len1+1;
    
    e = XT(k,1) - X(kk,1);
    n = XT(k,2) - X(kk,2);
    u = 0 - alt;

    H1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
          -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
          e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
      
    e = XT(k,3) - X(kk,1);
    n = XT(k,4) - X(kk,2);
    u = 0 - alt;

    H2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
          -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
          e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
      
    e = XT(k,5) - X(kk,1);
    n = XT(k,6) - X(kk,2);
    u = 0 - alt;

    H3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
          -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
          e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
    
    J1_dot = -J1*F1' - F1*J1 - J1*Q*J1 + H1'*invR*H1;
    J2_dot = -J2*F2' - F2*J2 - J2*Q*J2 + H2'*invR*H2;
    J3_dot = -J3*F3' - F3*J3 - J3*Q*J3 + H3'*invR*H3;
    
    Jp1_dot = -Jp1*F1' - F1*Jp1 - Jp1*Q*Jp1 + H1'*invR*H1;
    Jp2_dot = -Jp2*F2' - F2*Jp2 - Jp2*Q*Jp2 + H2'*invR*H2;
    Jp3_dot = -Jp3*F3' - F3*Jp3 - Jp3*Q*Jp3 + H3'*invR*H3;
    
    if (multisensor == 1)
        e = XT(k,1) - Pos(1);
        n = XT(k,2) - Pos(2);
        u = 0 - alt;
        H1_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos2(1);
        n = XT(k,2) - Pos2(2);
        u = 0 - alt;
        H1_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,1) - Pos3(1);
        n = XT(k,2) - Pos3(2);
        u = 0 - alt;
        H1_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H1 = [H1_1; H1_2; H1_3];
        
        e = XT(k,3) - Pos(1);
        n = XT(k,4) - Pos(2);
        u = 0 - alt;
        H2_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos2(1);
        n = XT(k,4) - Pos2(2);
        u = 0 - alt;
        H2_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,3) - Pos3(1);
        n = XT(k,4) - Pos3(2);
        u = 0 - alt;
        H2_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H2 = [H2_1; H2_2; H2_3];
        
        e = XT(k,5) - Pos(1);
        n = XT(k,6) - Pos(2);
        u = 0 - alt;
        H3_1 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos2(1);
        n = XT(k,6) - Pos2(2);
        u = 0 - alt;
        H3_2 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        e = XT(k,5) - Pos3(1);
        n = XT(k,6) - Pos3(2);
        u = 0 - alt;
        H3_3 = [n/(e^2+n^2), -e/(e^2+n^2), 0, 0;
              -e*u/(e^2+n^2+u^2)/sqrt(e^2+n^2), -u*n/(e^2+n^2+u^2)/sqrt(e^2+n^2), 0, 0;
              e/sqrt(e^2+n^2+u^2), n/sqrt(e^2+n^2+u^2), 0, 0];
          
        H3 = [H3_1; H3_2; H3_3];
              
              J1_dot = J1_dot + H1'*invRm/ratio*H1;
              J2_dot = J2_dot + H2'*invRm/ratio*H2;
              J3_dot = J3_dot + H3'*invRm/ratio*H3;
              
              Jp1_dot = Jp1_dot + H1'*invRm/ratio*H1;
              Jp2_dot = Jp2_dot + H2'*invRm/ratio*H2;
              Jp3_dot = Jp3_dot + H3'*invRm/ratio*H3;
    end
    
    J1 = J1 + J1_dot*dt2;
    J2 = J2 + J2_dot*dt2;
    J3 = J3 + J3_dot*dt2;
    
    I1(k+1) = -0.5*log(det(J1));
    I2(k+1) = -0.5*log(det(J2));
    I3(k+1) = -0.5*log(det(J3));
    
    Jp1 = Jp1 + Jp1_dot*dt2;
    Jp2 = Jp2 + Jp2_dot*dt2;
    Jp3 = Jp3 + Jp3_dot*dt2;
    
    Ip1 = -0.5*log(det(Jp1));
    Ip2 = -0.5*log(det(Jp2));
    Ip3 = -0.5*log(det(Jp3));
    I_tot_p(lenp+k+1) = Ip1 + Ip2 + Ip3;
    
    XT(k+1,:) = XT(k,:) + xt_dot*dt2;

end

I_tot = I1 + I2 + I3;

optimal = I_tot(end)

%%

time1 = dt1:dt1:tf1;
time = [time1 tf1+dt2:dt2:tf2];

figure(1);
plot(time(len1:len1+len2), I_tot(len1:len1+len2), 'r-.', 'linewidth', 2); hold on;
plot(timeg(len1:len1+len2g), Ig_best_tot(len1:len1+len2g), 'k--', 'linewidth', 2);
ylabel('Information Index', 'fontsize', 12);
xlabel('Time (s)', 'fontsize', 12);
legend('Reference', 'Optimal', 'Proposed');
grid on;
% axis([0 19.4 -23.6 -18]);

figure(4);
plot([time_p time], I_tot_p, 'r-.', 'linewidth', 2); hold on;
plot(timeg, Ig_best_tot, 'k--', 'linewidth', 2);
ylabel('Information Index', 'fontsize', 12);
xlabel('Time (s)', 'fontsize', 12);
legend('Reference', 'Optimal', 'Proposed');
grid on;
axis([-tp 25 -24 -12]);

figure(2);
plot([time_p time], I_tot_p, 'r-.', 'linewidth', 2); hold on;
plot(timeg, Ig_best_tot, 'k--', 'linewidth', 2);
ylabel('Information Index', 'fontsize', 12);
xlabel('Time (s)', 'fontsize', 12);
legend('Reference', 'Optimal', 'Proposed');
% axis([-2.6 19.4 -23.6 -17]);
grid on;

optimal_tp = I_tot_p(end);

XT = [res.solution.phase(1).state(:,4:9); res.solution.phase(2).state(:,4:9)];
X = [res.solution.phase(1).state(:,1:2); res.solution.phase(2).state(:,1:2)];

figure(3);
 hold on;
plot(Xg_best(len1:len1+len2g,1), Xg_best(len1:len1+len2g,2), 'k--', 'linewidth', 2.5);
plot(XT(:,1), XT(:,2), 'k:', 'linewidth', 1.5);
plot(X(:,1), X(:,2), 'r-.', 'linewidth', 2.5);
plot(XT(:,5), XT(:,6), 'k:', 'linewidth', 1.5);
plot(XT(:,3), XT(:,4), 'k:', 'linewidth', 1.5);

plot(XT(1,1), XT(1,2), 'k^');
plot(XT(1,3), XT(1,4), 'k^');
plot(XT(1,5), XT(1,6), 'k^');
plot(XT(end,1), XT(end,2), 'kd');
plot(XT(end,3), XT(end,4), 'kd');
plot(XT(end,5), XT(end,6), 'kd');
if (multisensor == 1)
   plot(Pos(1), Pos(2), 'kv', 'markersize', 9, 'markerface', [0.5, 0.9, 0.7]); 
   plot(Pos2(1), Pos2(2), 'kv', 'markersize', 9, 'markerface', [0.5, 0.9, 0.7]); 
   plot(Pos3(1), Pos3(2), 'kv', 'markersize', 9, 'markerface', [0.5, 0.9, 0.7]); 
end
axis([-10 80 -65 25]);


exp(optimal)/exp(reference)
exp(optimal_tp)/exp(reference)
exp(reference_tp)/exp(reference)
exp(proposed)/exp(reference)

exp(optimal)/exp(reference_tp)
exp(optimal_tp)/exp(reference_tp)
exp(reference)/exp(reference_tp)
exp(proposed)/exp(reference_tp)