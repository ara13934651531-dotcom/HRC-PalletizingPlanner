function drawPallet_v11(ax, pal, frm, fontName)
%DRAWPALLET_V11 绘制托盘 (半透明6面体+标签)
%  v19.1: 托盘底面在地面(Z=0), 顶面在surfZ (含支撑结构)
    x=frm.cx-pal.widthX/2; y=frm.cy-pal.depthY/2;
    w=pal.widthX; d=pal.depthY; h=pal.heightZ;
    % 托盘顶面Z (世界坐标): 优先使用surfZ, 回退到heightZ
    if isfield(pal, 'surfZ')
        zTop = pal.surfZ;
    else
        zTop = h;  % 回退: 旧行为
    end
    zBot = 0;  % v19.1: 托盘底面在地面 (含支撑腿/底座)
    v=[x y zBot;x+w y zBot;x+w y+d zBot;x y+d zBot;x y zTop;x+w y zTop;x+w y+d zTop;x y+d zTop];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',pal.color,'EdgeColor',[.15 .30 .60],'FaceAlpha',.45,'LineWidth',1.2);
    text(ax,frm.cx,frm.cy,zTop+0.03,sprintf('Pallet surf %.0fmm',zTop*1000),...
         'FontSize',12,'HorizontalAlignment','center','Color',[.05 .15 .45],...
         'FontWeight','bold','FontName',fontName);
end
