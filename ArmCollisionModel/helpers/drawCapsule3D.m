function drawCapsule3D(ax, p1, p2, r, col, alpha)
%DRAWCAPSULE3D 绘制胶囊体 (圆柱+两端半球)
    v = p2-p1; L = norm(v);
    if L < 1e-6, return; end
    [X,Y,Z] = cylinder(r, 12);
    Z = Z*L;
    dd=[0;0;1]; td=v(:)/L;
    cp = cross(dd,td);
    if norm(cp) > 1e-6
        RR=axang2r_local([cp'/norm(cp), acos(max(-1,min(1,dot(dd,td))))]);
    else
        RR=eye(3); if dot(dd,td)<0, RR(3,3)=-1; RR(1,1)=-1; end
    end
    for i=1:numel(X)
        pt=RR*[X(i);Y(i);Z(i)]; X(i)=pt(1)+p1(1); Y(i)=pt(2)+p1(2); Z(i)=pt(3)+p1(3);
    end
    surf(ax,X,Y,Z,'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
    [Xs,Ys,Zs]=sphere(8);
    surf(ax,Xs*r+p1(1),Ys*r+p1(2),Zs*r+p1(3),'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
    surf(ax,Xs*r+p2(1),Ys*r+p2(2),Zs*r+p2(3),'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
end
