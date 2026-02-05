function [T00_l,T01_l,T02_l,T03_l,T04_l,T05_l,T06_l, T07_l, T0T_l] = dual_arm_FKl(ql)
global d1 d4 d6 a2;
global T6T_l; % 定义工具系

%% 标准D-H参数（使用 q1 到 q6 代替 q(1) 到 q(6)）
DH=[0              0       0     -pi/2;
    ql(1)         d1       0     pi/2;
    ql(2)+pi/2     0      a2       pi;
    ql(3)+pi/2     0       0     pi/2;
    ql(4)         d4       0    -pi/2;
    ql(5)          0       0     pi/2;
    ql(6)         d6       0        0];
       
%% 齐次变换矩阵
T01_l=[cos(DH(1,1))   -sin(DH(1,1))*cos(DH(1,4))    sin(DH(1,1))*sin(DH(1,4))  DH(1,3)*cos(DH(1,1));
     sin(DH(1,1))    cos(DH(1,1))*cos(DH(1,4))   -cos(DH(1,1))*sin(DH(1,4))  DH(1,3)*sin(DH(1,1));
               0                  sin(DH(1,4))                 cos(DH(1,4))               DH(1,2);
               0                             0                            0                     1];
T12_l=[cos(DH(2,1))   -sin(DH(2,1))*cos(DH(2,4))    sin(DH(2,1))*sin(DH(2,4))  DH(2,3)*cos(DH(2,1));
     sin(DH(2,1))    cos(DH(2,1))*cos(DH(2,4))   -cos(DH(2,1))*sin(DH(2,4))  DH(2,3)*sin(DH(2,1));
               0                  sin(DH(2,4))                 cos(DH(2,4))               DH(2,2);
               0                             0                            0                     1];
T23_l=[cos(DH(3,1))   -sin(DH(3,1))*cos(DH(3,4))    sin(DH(3,1))*sin(DH(3,4))  DH(3,3)*cos(DH(3,1));
     sin(DH(3,1))    cos(DH(3,1))*cos(DH(3,4))   -cos(DH(3,1))*sin(DH(3,4))  DH(3,3)*sin(DH(3,1));
               0                  sin(DH(3,4))                 cos(DH(3,4))               DH(3,2);
               0                             0                            0                     1];
T34_l=[cos(DH(4,1))   -sin(DH(4,1))*cos(DH(4,4))    sin(DH(4,1))*sin(DH(4,4))  DH(4,3)*cos(DH(4,1));
     sin(DH(4,1))    cos(DH(4,1))*cos(DH(4,4))   -cos(DH(4,1))*sin(DH(4,4))  DH(4,3)*sin(DH(4,1));
               0                  sin(DH(4,4))                 cos(DH(4,4))               DH(4,2);
               0                             0                            0                     1];
T45_l=[cos(DH(5,1))   -sin(DH(5,1))*cos(DH(5,4))    sin(DH(5,1))*sin(DH(5,4))  DH(5,3)*cos(DH(5,1));
     sin(DH(5,1))    cos(DH(5,1))*cos(DH(5,4))   -cos(DH(5,1))*sin(DH(5,4))  DH(5,3)*sin(DH(5,1));
               0                  sin(DH(5,4))                 cos(DH(5,4))               DH(5,2);
               0                             0                            0                     1];
T56_l=[cos(DH(6,1))   -sin(DH(6,1))*cos(DH(6,4))    sin(DH(6,1))*sin(DH(6,4))  DH(6,3)*cos(DH(6,1));
     sin(DH(6,1))    cos(DH(6,1))*cos(DH(6,4))   -cos(DH(6,1))*sin(DH(6,4))  DH(6,3)*sin(DH(6,1));
               0                  sin(DH(6,4))                 cos(DH(6,4))               DH(6,2);
               0                             0                            0                     1];
T67_l=[cos(DH(7,1))   -sin(DH(7,1))*cos(DH(7,4))    sin(DH(7,1))*sin(DH(7,4))  DH(7,3)*cos(DH(7,1));
     sin(DH(7,1))    cos(DH(7,1))*cos(DH(7,4))   -cos(DH(7,1))*sin(DH(7,4))  DH(7,3)*sin(DH(7,1));
               0                  sin(DH(7,4))                 cos(DH(7,4))               DH(7,2);
               0                             0                            0                     1]; 

%% 计算正运动学
T00_l = eye(4);
T02_l = T01_l*T12_l;
T03_l = T01_l*T12_l*T23_l;  
T04_l = T01_l*T12_l*T23_l*T34_l;  
T05_l = T01_l*T12_l*T23_l*T34_l*T45_l;  
T06_l = T01_l*T12_l*T23_l*T34_l*T45_l*T56_l;  
T07_l = T01_l*T12_l*T23_l*T34_l*T45_l*T56_l*T67_l; 
T0T_l = T07_l*T6T_l; 