function [ R, the ] = Carte2Polar( X0 , X1 )
%CARTE2POLAR 이 함수의 요약 설명 위치
%   자세한 설명 위치

R = norm(X1 - X0);
the = atan2(X1(2)-X0(2), X1(1)-X0(1));

end

