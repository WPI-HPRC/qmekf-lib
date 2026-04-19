function skewMatrix = skewSymmetric(vec)
%VEC2CROSS Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    vec
end

arguments (Output)
    skewMatrix
end

skewMatrix = [0, -1.0 * vec(3), vec(2);
                    vec(3), 0, -1.0 * vec(1);
                    -1.0 * vec(2), vec(1), 0];
end