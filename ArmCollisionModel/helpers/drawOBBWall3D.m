function drawOBBWall3D(ax, center, dims, col, alpha)
%DRAWOBBWALL3D 绘制 Lozenge OBB 墙面板 (退化为扁平矩形面片)
%   center: 面板中心 (世界坐标, m)
%   dims: [xLen, yLen, zLen] (m) — 最薄轴自动作为法线
    hx = dims(1)/2; hy = dims(2)/2; hz = dims(3)/2;
    [~, thinAxis] = min(dims);
    switch thinAxis
        case 1  % YZ面板 (薄X) — 左/右墙
            x = center(1) * [1 1 1 1];
            y = center(2) + [-hy, hy, hy, -hy];
            z = center(3) + [-hz, -hz, hz, hz];
        case 2  % XZ面板 (薄Y) — 后墙
            x = center(1) + [-hx, hx, hx, -hx];
            y = center(2) * [1 1 1 1];
            z = center(3) + [-hz, -hz, hz, hz];
        case 3  % XY面板 (薄Z) — 地板/天花板
            x = center(1) + [-hx, hx, hx, -hx];
            y = center(2) + [-hy, hy, hy, -hy];
            z = center(3) * [1 1 1 1];
    end
    patch(ax, 'XData', x, 'YData', y, 'ZData', z, ...
        'FaceColor', col, 'FaceAlpha', alpha, ...
        'EdgeColor', col*0.7, 'EdgeAlpha', min(alpha*3, 0.4), 'LineWidth', 0.5);
end
