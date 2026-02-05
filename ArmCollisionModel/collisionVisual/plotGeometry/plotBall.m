function pointCenter = plotBall(T,x,y,z,R)
% ----------------------------- 绘制球体 -------------------------------
TNT = [1 0 0 x;
       0 1 0 y;
       0 0 1 z;
       0 0 0 1];
T0N = T;
T0T = T0N * TNT;
[X1,Y1,Z1] = sphere;
surf(X1 * R+T0T(1,4),Y1 * R+T0T(2,4),Z1 * R+T0T(3,4), 'FaceAlpha', 0.3);

pointCenter = [T0T(1,4), T0T(2,4), T0T(3,4)];

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