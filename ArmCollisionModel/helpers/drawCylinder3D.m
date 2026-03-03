function drawCylinder3D(ax, p1, p2, r, col, alpha)
%DRAWCYLINDER3D 绘制碰撞体圆柱 (无两端半球端盖)
    v = p2-p1; L = norm(v);
    if L < 1e-6, return; end
    [X,Y,Z] = cylinder(r, 16);
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
    surf(ax,X,Y,Z,'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none',...
        'FaceLighting','gouraud','AmbientStrength',0.4);
end
