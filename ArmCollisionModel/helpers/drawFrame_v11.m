function drawFrame_v11(ax, f, cylN)
%DRAWFRAME_V11 绘制框架 (4立柱 + 横梁 + 网格面板, 近端面开放入口)
    r=f.tubeR; wx=f.widthX; dy=f.depthY; h=f.height; cx=f.cx; cy=f.cy;
    % corners: 1=left-near, 2=right-near, 3=right-far, 4=left-far
    c=[cx-wx/2 cy-dy/2; cx+wx/2 cy-dy/2; cx+wx/2 cy+dy/2; cx-wx/2 cy+dy/2];
    % 4根立柱
    for i=1:4, drawTube_v11(ax,c(i,1),c(i,2),0,c(i,1),c(i,2),h,r,f.color,cylN); end
    rb = r*0.8;  % 横梁半径
    % 近端面 (Y-neg): 底边 + 顶边 (顶边为结构梁, 入口在下方开放)
    drawTube_v11(ax,c(1,1),c(1,2),0.05,c(2,1),c(2,2),0.05,rb,f.color,cylN);
    drawTube_v11(ax,c(1,1),c(1,2),h-0.05,c(2,1),c(2,2),h-0.05,rb,f.color,cylN);  % 前X顶梁
    % 远端面 (Y-pos): 底边 + 顶边
    drawTube_v11(ax,c(3,1),c(3,2),0.05,c(4,1),c(4,2),0.05,rb,f.color,cylN);
    drawTube_v11(ax,c(3,1),c(3,2),h-0.05,c(4,1),c(4,2),h-0.05,rb,f.color,cylN);
    % 顶部: 2根侧梁 (平行于Y轴, 连接近端→远端)
    drawTube_v11(ax,c(1,1),c(1,2),h-0.05,c(4,1),c(4,2),h-0.05,rb,f.color,cylN);  % 左侧
    drawTube_v11(ax,c(2,1),c(2,2),h-0.05,c(3,1),c(3,2),h-0.05,rb,f.color,cylN);  % 右侧
    % 底部: 2根侧梁 (平行于Y轴, 连接近端→远端底部)
    drawTube_v11(ax,c(1,1),c(1,2),0.05,c(4,1),c(4,2),0.05,rb,f.color,cylN);  % 左侧底
    drawTube_v11(ax,c(2,1),c(2,2),0.05,c(3,1),c(3,2),0.05,rb,f.color,cylN);  % 右侧底
    % --- 网格防护面板 (半透明, 后/左/右三面, 匹配Lozenge OBB碰撞体) ---
    panelAlpha = 0.12;
    meshColor = [0.6, 0.75, 0.9];
    zLo = 0.05; zHi = h - 0.05;
    % 后面板 (c4→c3, Y=cy+dy/2)
    patch(ax, [c(4,1) c(3,1) c(3,1) c(4,1)], ...
              [c(4,2) c(3,2) c(3,2) c(4,2)], ...
              [zLo zLo zHi zHi], meshColor, 'FaceAlpha', panelAlpha, 'EdgeColor', meshColor*0.7, 'EdgeAlpha', 0.3);
    % 左面板 (c1→c4, X=cx-wx/2)
    patch(ax, [c(1,1) c(4,1) c(4,1) c(1,1)], ...
              [c(1,2) c(4,2) c(4,2) c(1,2)], ...
              [zLo zLo zHi zHi], meshColor, 'FaceAlpha', panelAlpha, 'EdgeColor', meshColor*0.7, 'EdgeAlpha', 0.3);
    % 右面板 (c2→c3, X=cx+wx/2)
    patch(ax, [c(2,1) c(3,1) c(3,1) c(2,1)], ...
              [c(2,2) c(3,2) c(3,2) c(2,2)], ...
              [zLo zLo zHi zHi], meshColor, 'FaceAlpha', panelAlpha, 'EdgeColor', meshColor*0.7, 'EdgeAlpha', 0.3);
end
