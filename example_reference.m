function example_reference
% Deterministic geometry and packing example; see docs/running.md.
% Research context and citation: README.md#citation.

origin = [20, 0];
point = [23, 4];
[radius, angle] = Carte2Polar(origin, point);
assert(abs(radius - 5) < 1e-12);
assert(abs(angle - atan2(4, 3)) < 1e-12);
assert(norm(Polar2Carte(origin, radius, angle) - point) < 1e-12);

normalized_length = Dubins_RSL_length(0, 0, 4);
assert(abs(normalized_length - 4) < 1e-12);

matrix = [1, 2, 3; 4, 5, 6];
row_vector = mat2vec(matrix, 'row');
col_vector = mat2vec(matrix, 'col');
assert(isequal(row_vector, [1, 2, 3, 4, 5, 6]));
assert(isequal(col_vector, [1; 4; 2; 5; 3; 6]));
assert(isequal(vec2mat(row_vector, 2, 3, 'row'), matrix));
assert(isequal(vec2mat(col_vector, 2, 3, 'col'), matrix));

fprintf('radius = %.12f, angle = %.12f rad\n', radius, angle);
fprintf('normalized RSL length = %.12f\n', normalized_length);
fprintf('Reference example passed.\n');
end
