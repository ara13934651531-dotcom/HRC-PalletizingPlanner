function drawCabinet_v11(ax, c, bx, by)
%DRAWCABINET_V11 绘制电箱 (6面体)
    x=bx-c.widthX/2; y=by-c.depthY/2; w=c.widthX; d=c.depthY; h=c.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',c.color,'EdgeColor',[.5 .5 .5],'FaceAlpha',.95,'LineWidth',1);
end
