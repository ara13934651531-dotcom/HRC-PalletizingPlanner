function R = makeRotZ(a)
%MAKEROTZ 绕Z轴旋转4x4矩阵
    c=cos(a); s=sin(a);
    R=[c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end
