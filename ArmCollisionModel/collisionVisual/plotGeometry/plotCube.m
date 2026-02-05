function plotCube(x,y,z,length,width,height)
% ----------------------------- 绘制棱体 -------------------------------
vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
cuboidSize = [length width height];
start_point = [(x-length/2) (y-width/2) (z-height/2)];
vertex=repmat(start_point,8,1)+vertexIndex.*repmat(cuboidSize,8,1);
facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
color=[0;0;0;0;1;1;1;1];
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5);
view([1,1,1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on 
end