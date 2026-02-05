function [T00_r,T01_r,T02_r,T03_r,T04_r,T05_r,T06_r, T07_r, T0T_r]=dual_arm_FKr(qr)
global d1 d4 d6 a2;
global T6T_r; % 定义工具系
%% 标准D-H参数
DH=[0              0         0   pi/2;
    qr(1)         d1       0     pi/2;
    qr(2)+pi/2     0      a2       pi;
    qr(3)+pi/2     0       0     pi/2;
    qr(4)         d4       0    -pi/2;
    qr(5)          0       0     pi/2;
    qr(6)         d6       0        0];
       
%% 齐次变换矩阵
T01_r=[cos(DH(1,1))   -sin(DH(1,1))*cos(DH(1,4))    sin(DH(1,1))*sin(DH(1,4))  DH(1,3)*cos(DH(1,1));
     sin(DH(1,1))    cos(DH(1,1))*cos(DH(1,4))   -cos(DH(1,1))*sin(DH(1,4))  DH(1,3)*sin(DH(1,1));
               0                  sin(DH(1,4))                 cos(DH(1,4))               DH(1,2);
               0                             0                            0                     1];
T12_r=[cos(DH(2,1))   -sin(DH(2,1))*cos(DH(2,4))    sin(DH(2,1))*sin(DH(2,4))  DH(2,3)*cos(DH(2,1));
     sin(DH(2,1))    cos(DH(2,1))*cos(DH(2,4))   -cos(DH(2,1))*sin(DH(2,4))  DH(2,3)*sin(DH(2,1));
               0                  sin(DH(2,4))                 cos(DH(2,4))               DH(2,2);
               0                             0                            0                     1];
T23_r=[cos(DH(3,1))   -sin(DH(3,1))*cos(DH(3,4))    sin(DH(3,1))*sin(DH(3,4))  DH(3,3)*cos(DH(3,1));
     sin(DH(3,1))    cos(DH(3,1))*cos(DH(3,4))   -cos(DH(3,1))*sin(DH(3,4))  DH(3,3)*sin(DH(3,1));
               0                  sin(DH(3,4))                 cos(DH(3,4))               DH(3,2);
               0                             0                            0                     1];
T34_r=[cos(DH(4,1))   -sin(DH(4,1))*cos(DH(4,4))    sin(DH(4,1))*sin(DH(4,4))  DH(4,3)*cos(DH(4,1));
     sin(DH(4,1))    cos(DH(4,1))*cos(DH(4,4))   -cos(DH(4,1))*sin(DH(4,4))  DH(4,3)*sin(DH(4,1));
               0                  sin(DH(4,4))                 cos(DH(4,4))               DH(4,2);
               0                             0                            0                     1];
T45_r=[cos(DH(5,1))   -sin(DH(5,1))*cos(DH(5,4))    sin(DH(5,1))*sin(DH(5,4))  DH(5,3)*cos(DH(5,1));
     sin(DH(5,1))    cos(DH(5,1))*cos(DH(5,4))   -cos(DH(5,1))*sin(DH(5,4))  DH(5,3)*sin(DH(5,1));
               0                  sin(DH(5,4))                 cos(DH(5,4))               DH(5,2);
               0                             0                            0                     1];
T56_r=[cos(DH(6,1))   -sin(DH(6,1))*cos(DH(6,4))    sin(DH(6,1))*sin(DH(6,4))  DH(6,3)*cos(DH(6,1));
     sin(DH(6,1))    cos(DH(6,1))*cos(DH(6,4))   -cos(DH(6,1))*sin(DH(6,4))  DH(6,3)*sin(DH(6,1));
               0                  sin(DH(6,4))                 cos(DH(6,4))               DH(6,2);
               0                             0                            0                     1]; 
T67_r=[cos(DH(7,1))   -sin(DH(7,1))*cos(DH(7,4))    sin(DH(7,1))*sin(DH(7,4))  DH(7,3)*cos(DH(7,1));
     sin(DH(7,1))    cos(DH(7,1))*cos(DH(7,4))   -cos(DH(7,1))*sin(DH(7,4))  DH(7,3)*sin(DH(7,1));
               0                  sin(DH(7,4))                 cos(DH(7,4))               DH(7,2);
               0                             0                            0                     1]; 

%% 计算正运动学
T00_r = eye(4);
T02_r = T01_r*T12_r;
T03_r = T01_r*T12_r*T23_r;
T04_r = T01_r*T12_r*T23_r*T34_r;
T05_r = T01_r*T12_r*T23_r*T34_r*T45_r;
T06_r = T01_r*T12_r*T23_r*T34_r*T45_r*T56_r;
T07_r = T01_r*T12_r*T23_r*T34_r*T45_r*T56_r*T67_r;
T0T_r = T07_r*T6T_r;
