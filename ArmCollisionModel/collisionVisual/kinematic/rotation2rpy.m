function [x,y,z,roll,pitch,yaw] = rotation2rpy(T)
% -------------------------- 齐次矩阵转xyzrpy ----------------------------
epsilon=1E-12;
pitch = atan2(-T(3,1), sqrt(T(1,1)*T(1,1)+T(2,1)*T(2,1)));
if (abs(pitch) > (pi/2.0-epsilon)) 
    yaw = atan2(-T(1,2), T(2,2));
    roll  = 0.0 ;
else 
    roll  = atan2(T(3,2), T(3,3));
    yaw   = atan2(T(2,1), T(1,1));
end
x = T(1,4);
y = T(2,4);
z = T(3,4);
end