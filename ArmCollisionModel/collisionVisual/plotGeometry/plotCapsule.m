function [t1, t2] = plotCapsule(T,x1,y1,z1,x2,y2,z2,R)
% ----------------------------- 绘制胶囊体 -------------------------------
H = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
TNT = vector2rotation(x1,y1,z1,x2,y2,z2);
T0N = T;
T0T = T0N * TNT;

t1 = pTransform(T, [x1,y1,z1]);
t2 = pTransform(T, [x2,y2,z2]);

t = R:-1e-2:0;
r = sqrt(R*R - t.*t);
[X1,Y1,Z1] = cylinder(r);
[X2,Y2,Z2] = cylinder(R);
t = 0:1e-2:R;
r = sqrt(R*R - t.*t);
[X3,Y3,Z3] = cylinder(r);
X = [X1;X2;X3];
Y = [Y1;Y2;Y3];
Z = [Z1*R;Z2*H+R;Z3*R+H+R];
p0 = [0;0;(R+H/2)];
pn = [T0T(1,4);T0T(2,4);T0T(3,4)];
transP = pn - p0;
s = surf(X+transP(1),Y+transP(2),Z+transP(3));
origin = [pn(1),pn(2),pn(3)];
[~,~,~,roll,pitch,yaw] = rotation2rpy(T0T);
roll = roll*180/pi;  
pitch = pitch*180/pi;
yaw = yaw*180/pi;

rotate(s,[1 0 0],roll,origin)
rotate(s,[0 1 0],pitch,origin)
rotate(s,[0 0 1],yaw,origin)

s.FaceAlpha = 0.2;
s.FaceColor = 'flat';

quiver3(T(1,4),T(2,4),T(3,4),...
        T(1,3)/5,T(2,3)/5,T(3,3)/5,...
        'LineWidth',2,'Color','b');
quiver3(T(1,4),T(2,4),T(3,4),...
        T(1,1)/5,T(2,1)/5,T(3,1)/5,...
        'LineWidth',2,'Color','r');
quiver3(T(1,4),T(2,4),T(3,4),...
        T(1,2)/5,T(2,2)/5,T(3,2)/5,...
        'LineWidth',2,'Color','g');

xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
axis equal
end

function tp = pTransform(T, p)
    t = [T(1,4);T(2,4);T(3,4)];
    R = T(1:3, 1:3);
    tp = R*p' + t;
end

function [T] = vector2rotation(x1,y1,z1,x2,y2,z2)
v = [(x2-x1);(y2-y1);(z2-z1)];
v_unit = v / norm(v);
alpha = 0; 
beta = atan2(v_unit(1),v_unit(3)); 
gama = asin(-v_unit(2));
ca = cos(alpha); sa = sin(alpha);
cb = cos(beta);sb = sin(beta);
cc = cos(gama);sc = sin(gama);
A = [ca*cb  ca*sb*sc - sa*cc  ca*sb*cc + sa*sc;
     sa*cb  sa*sb*sc + ca*cc  sa*sb*cc - ca*sc;
       -sb             cb*sc             cb*cc];
B = [(x1+x2)/2; (y1+y2)/2; (z1+z2)/2];
T = [A B;0 0 0 1];
end