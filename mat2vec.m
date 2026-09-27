function [ vec ] = mat2vec( mat, mode )
%MAT2VEC 이 함수의 요약 설명 위치
%   matrix to vector

[r c] = size(mat);

% matrix to colum vector
if (strcmp(mode, 'col') == 1)
   
    vec = [];
    for k = 1:1:c;
        vec = [vec; mat(:,k)];
    end
    
% matrix to row vector
else

    if (strcmp(mode, 'row') ~= 1)
        display('mat2vec: wrong mode input. default: row');
    end
    
    vec = [];
    for k = 1:1:r
        vec = [vec mat(k,:)];
    end

end

