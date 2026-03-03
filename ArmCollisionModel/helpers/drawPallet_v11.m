function drawPallet_v11(ax, pal, frm, fontName)
%DRAWPALLET_V11 绘制托盘 (半透明6面体+标签)
    x=frm.cx-pal.widthX/2; y=frm.cy-pal.depthY/2;
    w=pal.widthX; d=pal.depthY; h=pal.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',pal.color,'EdgeColor',[.15 .30 .60],'FaceAlpha',.45,'LineWidth',1.2);
    text(ax,frm.cx,frm.cy,h+0.03,sprintf('Pallet %dcm',round(h*100)),...
         'FontSize',12,'HorizontalAlignment','center','Color',[.05 .15 .45],...
         'FontWeight','bold','FontName',fontName);
end
