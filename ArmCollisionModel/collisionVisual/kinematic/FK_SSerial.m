function [T00,T01,T02,T03,T04,T05,T0T]=FK_SSerial(q)
global d1 d2 d3 d4 d5 d6 a2 a3;
global T6T; % 定义工具系
%% 标准D-H参数
DH=[q(1)         d1             0      pi/2;
    q(2)         0              a2       0;
    q(3)         0              a3       0;
    q(4)         d2-d3+d4       0      pi/2;
    q(5)         d5             0     -pi/2;
    q(6)         d6             0        0];
       
%% 齐次变换矩阵
T01=[cos(DH(1,1))   -sin(DH(1,1))*cos(DH(1,4))    sin(DH(1,1))*sin(DH(1,4))  DH(1,3)*cos(DH(1,1));
     sin(DH(1,1))    cos(DH(1,1))*cos(DH(1,4))   -cos(DH(1,1))*sin(DH(1,4))  DH(1,3)*sin(DH(1,1));
               0                  sin(DH(1,4))                 cos(DH(1,4))               DH(1,2);
               0                             0                            0                     1];
T12=[cos(DH(2,1))   -sin(DH(2,1))*cos(DH(2,4))    sin(DH(2,1))*sin(DH(2,4))  DH(2,3)*cos(DH(2,1));
     sin(DH(2,1))    cos(DH(2,1))*cos(DH(2,4))   -cos(DH(2,1))*sin(DH(2,4))  DH(2,3)*sin(DH(2,1));
               0                  sin(DH(2,4))                 cos(DH(2,4))               DH(2,2);
               0                             0                            0                     1];
T23=[cos(DH(3,1))   -sin(DH(3,1))*cos(DH(3,4))    sin(DH(3,1))*sin(DH(3,4))  DH(3,3)*cos(DH(3,1));
     sin(DH(3,1))    cos(DH(3,1))*cos(DH(3,4))   -cos(DH(3,1))*sin(DH(3,4))  DH(3,3)*sin(DH(3,1));
               0                  sin(DH(3,4))                 cos(DH(3,4))               DH(3,2);
               0                             0                            0                     1];
T34=[cos(DH(4,1))   -sin(DH(4,1))*cos(DH(4,4))    sin(DH(4,1))*sin(DH(4,4))  DH(4,3)*cos(DH(4,1));
     sin(DH(4,1))    cos(DH(4,1))*cos(DH(4,4))   -cos(DH(4,1))*sin(DH(4,4))  DH(4,3)*sin(DH(4,1));
               0                  sin(DH(4,4))                 cos(DH(4,4))               DH(4,2);
               0                             0                            0                     1];
T45=[cos(DH(5,1))   -sin(DH(5,1))*cos(DH(5,4))    sin(DH(5,1))*sin(DH(5,4))  DH(5,3)*cos(DH(5,1));
     sin(DH(5,1))    cos(DH(5,1))*cos(DH(5,4))   -cos(DH(5,1))*sin(DH(5,4))  DH(5,3)*sin(DH(5,1));
               0                  sin(DH(5,4))                 cos(DH(5,4))               DH(5,2);
               0                             0                            0                     1];
T56=[cos(DH(6,1))   -sin(DH(6,1))*cos(DH(6,4))    sin(DH(6,1))*sin(DH(6,4))  DH(6,3)*cos(DH(6,1));
     sin(DH(6,1))    cos(DH(6,1))*cos(DH(6,4))   -cos(DH(6,1))*sin(DH(6,4))  DH(6,3)*sin(DH(6,1));
               0                  sin(DH(6,4))                 cos(DH(6,4))               DH(6,2);
               0                             0                            0                     1]; 

%% 计算正运动学
T00 = eye(4);
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;
T0T = T01*T12*T23*T34*T45*T56*T6T;