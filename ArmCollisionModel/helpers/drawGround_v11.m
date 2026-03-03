function drawGround_v11(ax, x0, x1, y0, y1)
%DRAWGROUND_V11 绘制地面矩形
    patch(ax,'Vertices',[x0 y0 0;x1 y0 0;x1 y1 0;x0 y1 0],'Faces',[1 2 3 4],...
          'FaceColor',[.92 .92 .90],'EdgeColor','none','FaceAlpha',0.4);
end
