function T = makeTrans(xyz)
%MAKETRANS 4x4 平移矩阵
    T = eye(4); T(1:3,4) = xyz(:);
end
