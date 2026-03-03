function drawTube_v11(ax, x1, y1, z1, x2, y2, z2, radius, color, cylN)
%DRAWTUBE_V11 绘制管子/横梁 (无端盖)
    [X,Y,Z]=cylinder(radius,cylN);
    v=[x2-x1;y2-y1;z2-z1]; l=norm(v);
    if l<.001, return; end
    Z=Z*l; dd=[0;0;1]; td=v/l; cp=cross(dd,td);
    if norm(cp)>1e-6
        RR=axang2r_local([cp'/norm(cp),acos(max(-1,min(1,dot(dd,td))))]);
    else
        RR=eye(3); if dot(dd,td)<0, RR(3,3)=-1; RR(1,1)=-1; end
    end
    for i=1:numel(X)
        pt=RR*[X(i);Y(i);Z(i)]; X(i)=pt(1)+x1; Y(i)=pt(2)+y1; Z(i)=pt(3)+z1;
    end
    surf(ax,X,Y,Z,'FaceColor',color,'EdgeColor','none','FaceAlpha',.85);
end
