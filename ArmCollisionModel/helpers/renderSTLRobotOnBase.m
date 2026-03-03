function renderSTLRobotOnBase(ax, meshData, JOINTS, q_rad, colors, alpha, Tbase)
%RENDERSTLROBOTONBASE 渲染STL网格机器人 (含基座变换)
    hold(ax,'on');
    T_all = urdfFK(JOINTS, q_rad);
    for li = 1:7
        V = meshData{li}.V; F = meshData{li}.F;
        T = Tbase * T_all{li}; R = T(1:3,1:3); t = T(1:3,4);
        V_w = (R * V' + t)';
        patch(ax,'Faces',F,'Vertices',V_w,'FaceColor',colors{li},'EdgeColor','none',...
            'FaceAlpha',alpha,'FaceLighting','gouraud','AmbientStrength',0.4,...
            'DiffuseStrength',0.7,'SpecularStrength',0.3);
    end
    tw = Tbase * T_all{8};
    plot3(ax,tw(1,4),tw(2,4),tw(3,4),'rp','MarkerSize',10,'MarkerFaceColor','r');
end
