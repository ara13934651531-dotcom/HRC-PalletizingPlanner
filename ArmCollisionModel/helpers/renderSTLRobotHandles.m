function handles = renderSTLRobotHandles(ax, meshData, JOINTS, q_rad, colors, alpha, Tbase)
%RENDERSTLROBOTHANDLES 渲染STL网格机器人并返回图形句柄 (可更新)
    T_all = urdfFK(JOINTS, q_rad);
    handles = gobjects(9,1);
    for li = 1:7
        V = meshData{li}.V; F = meshData{li}.F;
        T = Tbase * T_all{li}; R = T(1:3,1:3); t = T(1:3,4);
        V_w = (R * V' + t)';
        handles(li) = patch(ax,'Faces',F,'Vertices',V_w,'FaceColor',colors{li},...
            'EdgeColor','none','FaceAlpha',alpha,'FaceLighting','gouraud',...
            'AmbientStrength',0.4,'DiffuseStrength',0.7,'SpecularStrength',0.3);
    end
    tw = Tbase * T_all{8};
    handles(8) = plot3(ax,tw(1,4),tw(2,4),tw(3,4),'rp','MarkerSize',10,'MarkerFaceColor','r');
    jpts = zeros(7,3);
    for i=1:7, jw = Tbase*T_all{i}; jpts(i,:) = jw(1:3,4)'; end
    handles(9) = plot3(ax,jpts(:,1),jpts(:,2),jpts(:,3),'k-o','MarkerSize',4,...
        'MarkerFaceColor',[0.3 0.3 0.3],'LineWidth',1);
end
