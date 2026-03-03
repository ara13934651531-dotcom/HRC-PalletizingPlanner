function drawFrame_v11(ax, f, cylN)
%DRAWFRAME_V11 绘制框架 (4立柱 + 横梁, 近端面开放)
    r=f.tubeR; wx=f.widthX; dy=f.depthY; h=f.height; cx=f.cx; cy=f.cy;
    % corners: 1=left-near, 2=right-near, 3=right-far, 4=left-far
    c=[cx-wx/2 cy-dy/2; cx+wx/2 cy-dy/2; cx+wx/2 cy+dy/2; cx-wx/2 cy+dy/2];
    % 4根立柱
    for i=1:4, drawTube_v11(ax,c(i,1),c(i,2),0,c(i,1),c(i,2),h,r,f.color,cylN); end
    rb = r*0.8;  % 横梁半径
    % 近端面 (Y-neg): 仅底边 (开放入口, 无顶边/中间栏杆, 便于机器人进出)
    drawTube_v11(ax,c(1,1),c(1,2),0.05,c(2,1),c(2,2),0.05,rb,f.color,cylN);
    % 远端面 (Y-pos): 底边 + 顶边
    drawTube_v11(ax,c(3,1),c(3,2),0.05,c(4,1),c(4,2),0.05,rb,f.color,cylN);
    drawTube_v11(ax,c(3,1),c(3,2),h-0.05,c(4,1),c(4,2),h-0.05,rb,f.color,cylN);
    % 顶部: 2根侧梁 (平行于Y轴, 连接近端→远端)
    drawTube_v11(ax,c(1,1),c(1,2),h-0.05,c(4,1),c(4,2),h-0.05,rb,f.color,cylN);  % 左侧
    drawTube_v11(ax,c(2,1),c(2,2),h-0.05,c(3,1),c(3,2),h-0.05,rb,f.color,cylN);  % 右侧
    % 底部: 2根侧梁 (平行于Y轴, 连接近端→远端底部)
    drawTube_v11(ax,c(1,1),c(1,2),0.05,c(4,1),c(4,2),0.05,rb,f.color,cylN);  % 左侧底
    drawTube_v11(ax,c(2,1),c(2,2),0.05,c(3,1),c(3,2),0.05,rb,f.color,cylN);  % 右侧底
end
