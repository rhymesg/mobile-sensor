

clc;
clear all;

J = eye(4);

F = [zeros(2,2), eye(2); zeros(2,4)];

Q = [zeros(2,4); zeros(2,2), 0.25*eye(2)];

H = [eye(2), zeros(2,2)];

R = 0.01*eye(2);

I = 0;
for k = 1:1:10;
J = J + (-J*F'-F*J-J*Q*J + I*H'*inv(R)*H)
det(J)
end
