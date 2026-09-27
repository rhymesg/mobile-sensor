function [ X1 ] = Polar2Carte( X0, R, the )
%POLAR2CARTE 이 함수의 요약 설명 위치
%   자세한 설명 위치

X1(1) = X0(1) + R*cos(the);
X1(2) = X0(2) + R*sin(the);

end

