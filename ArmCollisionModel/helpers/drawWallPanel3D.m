function drawWallPanel3D(ax, p1, p2, r, col, alpha)
%DRAWWALLPANEL3D 绘制墙面板碰撞体为扁平半透明矩形面板
%   替代厚圆柱, 消除端面圆形伪影, 不遮蔽框架立柱
%   面板在包含胶囊中心线和Z轴的垂直平面内:
%     宽度 = 胶囊长度 (p1→p2), 高度 = 2*r (胶囊直径=碰撞覆盖范围)
%   p1, p2: 胶囊中心线端点 (世界坐标, m)
%   r: 胶囊半径 (面板半高, m)
    x = [p1(1), p2(1), p2(1), p1(1)];
    y = [p1(2), p2(2), p2(2), p1(2)];
    z = [p1(3)-r, p2(3)-r, p2(3)+r, p1(3)+r];
    patch(ax, 'XData', x, 'YData', y, 'ZData', z, ...
        'FaceColor', col, 'FaceAlpha', alpha, ...
        'EdgeColor', col, 'EdgeAlpha', min(alpha*2.5, 0.5), 'LineWidth', 0.5);
end
