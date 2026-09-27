function [ mat ] = vec2mat( vec, r, c, mode )
%VEC2MAT 이 함수의 요약 설명 위치
%   자세한 설명 위치

mat = zeros(r,c);

% colum vector to matrix
if (strcmp(mode, 'col') == 1)
    
    for k = 1:1:c;
        mat(:,k) = vec( (k-1)*r+1 : k*r );
    end
    
% row vector to matrix
else
    
    for k = 1:1:r;
        mat(k,:) = vec( (k-1)*c+1 : k*c );
    end
    
end


end

