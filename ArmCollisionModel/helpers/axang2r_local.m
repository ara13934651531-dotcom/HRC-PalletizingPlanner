function R = axang2r_local(ax)
%AXANG2R_LOCAL 轴角转旋转矩阵 (无需Robotics Toolbox)
    a=ax(1:3); g=ax(4); c=cos(g); s=sin(g); t=1-c;
    x=a(1); y=a(2); z=a(3);
    R=[t*x*x+c   t*x*y-s*z t*x*z+s*y;
       t*x*y+s*z t*y*y+c   t*y*z-s*x;
       t*x*z-s*y t*y*z+s*x t*z*z+c];
end
