function [handles, tcp_world] = renderCapsuleRobotHandles(ax, q_deg, baseOffset, ~)
%RENDERCAPSULEROBOTHANDLES 碰撞包络模型渲染 (v16.0: SO getUIInfoMation真实碰撞体)
%   当SO库可用时使用真实碰撞体位置; 否则回退到fk2Skeleton(近似, 弃用)
%   输入: q_deg=关节角(deg), baseOffset=[bx,by,bz] 基座世界坐标(m)
%   输出: handles=图形句柄, tcp_world=TCP世界坐标 [x,y,z](m)
    capsuleColors = {[0.4 0.4 0.45], [0.2 0.4 0.8], [0.2 0.7 0.3], ...
                     [0.9 0.5 0.1], [0.6 0.2 0.8]};
    capsuleAlpha = 0.45;
    bx = baseOffset(1); by = baseOffset(2); bz = baseOffset(3);
    
    if libisloaded('libHRCInterface')
        % ★ SO库可用: 使用getUIInfoMation获取真实碰撞体位置
        [joints, bodies] = soFK2Skeleton(q_deg);
        handles = gobjects(7,1);  % 5碰撞体 + TCP标记 + 骨架线
        for bi = 1:5
            if strcmp(bodies(bi).type, 'capsule')
                p1 = bodies(bi).p1 + [bx, by, bz];
                p2 = bodies(bi).p2 + [bx, by, bz];
                handles(bi) = drawCapsule3D_h(ax, p1, p2, bodies(bi).radius_m, capsuleColors{bi}, capsuleAlpha);
            elseif strcmp(bodies(bi).type, 'ball')
                c = bodies(bi).center + [bx, by, bz];
                [Xs,Ys,Zs] = sphere(8);
                handles(bi) = surf(ax, Xs*bodies(bi).radius_m+c(1), ...
                    Ys*bodies(bi).radius_m+c(2), Zs*bodies(bi).radius_m+c(3), ...
                    'FaceColor', capsuleColors{bi}, 'FaceAlpha', capsuleAlpha, 'EdgeColor', 'none');
            end
        end
        tcp_world = joints(5,:) + [bx, by, bz];
        handles(6) = plot3(ax, tcp_world(1), tcp_world(2), tcp_world(3),...
            'rp','MarkerSize',14,'MarkerFaceColor','r','LineWidth',1.5);
        skel_w = joints + [bx, by, bz];
        handles(7) = plot3(ax, skel_w(:,1), skel_w(:,2), skel_w(:,3), 'k--o',...
            'MarkerSize',5,'MarkerFaceColor',[0.3 0.3 0.3],'LineWidth',1.2);
    else
        % 回退: 旧fk2Skeleton近似 (⚠ v1.0.0后尺度不正确, 仅用于无SO场景)
        capsuleR = [0.160, 0.140, 0.120, 0.100];
        joints = fk2Skeleton(q_deg);
        joints_w = joints + [bx, by, bz];
        handles = gobjects(6,1);
        for li = 1:4
            handles(li) = drawCapsule3D_h(ax, joints_w(li,:), joints_w(li+1,:), capsuleR(li), capsuleColors{li}, capsuleAlpha);
        end
        tcp_world = joints_w(5,:);
        handles(5) = plot3(ax, tcp_world(1), tcp_world(2), tcp_world(3),...
            'rp','MarkerSize',14,'MarkerFaceColor','r','LineWidth',1.5);
        handles(6) = plot3(ax, joints_w(:,1), joints_w(:,2), joints_w(:,3), 'k--o',...
            'MarkerSize',5,'MarkerFaceColor',[0.3 0.3 0.3],'LineWidth',1.2);
    end
end
