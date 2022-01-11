% Cosine matrix
function [C] = C_from_to(from, to)
C = zeros(3);
for i = 1:size(from, 1)
    for j = 1:size(to, 1)
    C(i,j) = dot(from(:,i),to(:,j));
    end
end
disp("C = ")
disp(C)
end