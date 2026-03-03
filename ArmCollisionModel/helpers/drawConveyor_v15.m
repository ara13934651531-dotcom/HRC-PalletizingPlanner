function drawConveyor_v15(ax, cv, cylN)
%DRAWCONVEYOR_V15 绘制传送带 (侧板+腿+滚筒+皮带)
    cx=cv.cx; cy=cv.cy; ly=cv.lengthY; wx=cv.widthX; hz=cv.heightZ;
    x0=cx-wx/2; y0=cy-ly/2;
    % 侧板
    drawBox3D_v11(ax,x0-.015,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    drawBox3D_v11(ax,x0+wx,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    % 腿 (6对)
    lw=.035; nLegs=6;
    legSpacing = ly/(nLegs+1);
    for li=1:nLegs
        yLeg = y0 + li*legSpacing;
        for s=[-1 1]
            lx=cx+s*(wx/2-.06);
            drawBox3D_v11(ax,lx-lw/2,yLeg-lw/2,0,lw,lw,hz-.01,[.25 .25 .25]);
        end
    end
    % 滚筒
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        [X,Y,Z]=cylinder(cv.rollerR,cylN); Z=Z*wx*.9-wx*.9/2;
        surf(ax,Z+cx,zeros(size(X))+ry,X+hz+cv.rollerR,'FaceColor',[.55 .55 .55],'EdgeColor','none','FaceAlpha',.6);
    end
    % 皮带
    bz=hz+cv.rollerR;
    bV=[x0+.02 y0+.04 bz;x0+wx-.02 y0+.04 bz;x0+wx-.02 y0+ly-.04 bz;x0+.02 y0+ly-.04 bz;
        x0+.02 y0+.04 bz+cv.beltH;x0+wx-.02 y0+.04 bz+cv.beltH;
        x0+wx-.02 y0+ly-.04 bz+cv.beltH;x0+.02 y0+ly-.04 bz+cv.beltH];
    patch(ax,'Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[.15 .15 .15],'FaceAlpha',.92);
end
