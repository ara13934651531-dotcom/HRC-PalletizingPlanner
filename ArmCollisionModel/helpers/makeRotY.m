function R = makeRotY(a)
%MAKEROTY 绕Y轴旋转4x4矩阵
    c=cos(a); s=sin(a);
    R=[c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end
