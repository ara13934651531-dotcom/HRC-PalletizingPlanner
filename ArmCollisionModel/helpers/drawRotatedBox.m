function h = drawRotatedBox(ax, pos, bx, yaw_rad)
%DRAWROTATEDBOX 绘制绕Z轴旋转的箱子 (居中定位)
%   pos     = [cx, cy, cz] - 箱子体积中心 (m)
%   bx      = struct with .lx, .wy, .hz, .color - 箱子尺寸(m)和颜色
%   yaw_rad = 绕Z轴旋转角度 (rad), 0=无旋转 (与world X轴对齐)
%
%   返回: patch句柄
%
%   用途: TCP搬运箱子时, 箱子跟随TCP绕Z轴旋转, 用于避障可视化
%         TCP水平朝下时, J6旋转 ≈ 箱子水平旋转
%
%   @file   drawRotatedBox.m
%   @date   2026-02-25

    if nargin < 4 || isempty(yaw_rad), yaw_rad = 0; end

    hx = bx.lx / 2;
    hy = bx.wy / 2;
    hz = bx.hz / 2;

    % 8个顶点 (以原点为中心)
    corners = [
        -hx, -hy, -hz;
         hx, -hy, -hz;
         hx,  hy, -hz;
        -hx,  hy, -hz;
        -hx, -hy,  hz;
         hx, -hy,  hz;
         hx,  hy,  hz;
        -hx,  hy,  hz;
    ];

    % Z轴旋转矩阵
    c = cos(yaw_rad);
    s = sin(yaw_rad);
    Rz = [c, -s, 0;
          s,  c, 0;
          0,  0, 1];

    % 旋转 + 平移到世界坐标
    rotated = (Rz * corners')' + pos;

    % 6个面
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

    h = patch(ax, 'Vertices', rotated, 'Faces', faces, ...
        'FaceColor', bx.color, 'EdgeColor', [.35 .25 .1], ...
        'FaceAlpha', 0.95, 'LineWidth', 1.2);
end
