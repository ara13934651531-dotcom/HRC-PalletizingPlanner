function renderSTLRobot(ax, meshData, JOINTS, q_rad, colors, alpha)
%RENDERSTLROBOT 渲染STL网格机器人 (无基座偏移)
    hold(ax,'on');
    T_all = urdfFK(JOINTS, q_rad);
    for li = 1:7
        V = meshData{li}.V; F = meshData{li}.F;
        T = T_all{li}; R = T(1:3,1:3); t = T(1:3,4);
        V_w = (R * V' + t)';
        patch(ax,'Faces',F,'Vertices',V_w,'FaceColor',colors{li},'EdgeColor','none',...
            'FaceAlpha',alpha,'FaceLighting','gouraud','AmbientStrength',0.4,...
            'DiffuseStrength',0.7,'SpecularStrength',0.3);
    end
    tcp = T_all{8}(1:3,4)';
    plot3(ax,tcp(1),tcp(2),tcp(3),'rp','MarkerSize',10,'MarkerFaceColor','r');
    jpts = zeros(7,3);
    for i=1:7, jpts(i,:) = T_all{i}(1:3,4)'; end
    plot3(ax,jpts(:,1),jpts(:,2),jpts(:,3),'k-o','MarkerSize',4,'MarkerFaceColor',[0.3 0.3 0.3]);
    axis(ax,'equal'); grid(ax,'on'); camlight(ax,'headlight'); lighting(ax,'gouraud');
end
