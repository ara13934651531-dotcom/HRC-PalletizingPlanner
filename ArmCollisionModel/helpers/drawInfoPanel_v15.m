function drawInfoPanel_v15(ax, taskIdx, nTotal, q_deg, vel, tcp, dist, time_s, ...
    cjkFont, dColor, soActive, carrying, nPlaced, nBoxTotal, envColl, ...
    selfPair, envCollFrame, selfCollFlag)
%DRAWINFOPANEL_V15 绘制实时信息面板 (v6.2版, 含碰撞对+工具球状态+环境碰撞)
%
%  v6.2 新增参数:
%    selfPair     [1x2]: 自碰撞最近对 (如 [6,4]=工具球vs腕部)
%    envCollFrame logical: 当前帧环境碰撞 (区域约束)
%    selfCollFlag logical: 当前帧自碰撞标志
%
%  颜色规则:
%    搬运时 pair(6,4) 距离低 → 橙色 (工具球预期)
%    搬运时 pair≠(6,4) 距离低 → 红色 (危险, 箱子打臂体)
%    非搬运 距离低 → 红色 (自碰撞警告)
%
%  兼容旧调用: 不带 selfPair/envCollFrame/selfCollFlag 仍可工作

    % --- 参数兼容 (v6.1旧接口) ---
    if nargin < 16, selfPair = [0,0]; end
    if nargin < 17, envCollFrame = false; end
    if nargin < 18, selfCollFlag = false; end

    rectangle(ax,'Position',[0 0 1 1],'FaceColor',[0.97 0.97 0.99],...
        'EdgeColor',[0.5 0.5 0.7],'LineWidth',2,'Curvature',0.02);
    yP = 0.96;
    text(ax,0.5,yP,'v6.2 碰撞监控','FontSize',14,'FontWeight','bold',...
        'HorizontalAlignment','center','Color',[0.1 0.1 0.3],'FontName',cjkFont);
    yP=yP-0.03;
    if soActive, srcStr = 'libHRCInterface.so (实时)'; else, srcStr = 'C++ (预计算)'; end
    text(ax,0.5,yP,srcStr,'FontSize',9,'FontWeight','bold','HorizontalAlignment','center',...
        'Color',[0.3 0.3 0.5],'FontName',cjkFont);
    
    % --- 进度条 ---
    yP=yP-0.04;
    progW = max(0.01, 0.86*(taskIdx/nTotal));
    rectangle(ax,'Position',[0.05 yP-0.03 0.90 0.04],'FaceColor',[0.15 0.35 0.65],'EdgeColor','none','Curvature',0.3);
    rectangle(ax,'Position',[0.07 yP-0.025 progW 0.03],'FaceColor',[0.3 0.75 0.45],'EdgeColor','none','Curvature',0.3);
    text(ax,0.5,yP-0.01,sprintf('Task %d / %d  [FIFO]',taskIdx,nTotal),...
        'FontSize',12,'FontWeight','bold','HorizontalAlignment','center','Color','w','FontName',cjkFont);
    
    % === 自碰撞距离 (核心指标) ===
    yP=yP-0.055;
    isToolPair = carrying && any(selfPair==6) && any(selfPair==4);
    if isToolPair
        borderColor = [0.9 0.5 0.0];  % 橙色边框: 工具球预期低距离
    else
        borderColor = dColor;
    end
    rectangle(ax,'Position',[0.05 yP-0.035 0.90 0.05],'FaceColor',[1 1 1],...
        'EdgeColor',borderColor,'LineWidth',3,'Curvature',0.2);
    text(ax,0.5,yP-0.01,sprintf('Self: %.1f mm', dist),...
        'FontSize',14,'FontWeight','bold','HorizontalAlignment','center','Color',dColor,'FontName',cjkFont);
    
    % === 碰撞对详情 ===
    yP=yP-0.055;
    if soActive && any(selfPair ~= 0)
        if isToolPair
            % 搬运时 pair(6,4) = 工具球 vs 腕部 → 正常, 橙色
            pairStr = sprintf('pair(%d,%d) 工具球-腕部', selfPair(1), selfPair(2));
            pairColor = [0.7 0.45 0.0];
            text(ax,0.08,yP,pairStr,'FontSize',9,'FontWeight','bold','Color',pairColor,'FontName',cjkFont);
            yP=yP-0.02;
            text(ax,0.08,yP,sprintf('  z=-400 r=120mm | 裕度: %.0fmm', dist),...
                'FontSize',8,'Color',[0.4 0.5 0.3],'FontName',cjkFont);
        elseif selfCollFlag
            % 真实自碰撞 → 红色警告
            pairStr = sprintf('pair(%d,%d) 碰撞! dist=%.1fmm', selfPair(1), selfPair(2), dist);
            pairColor = [0.9 0.1 0.1];
            text(ax,0.08,yP,pairStr,'FontSize',10,'FontWeight','bold','Color',pairColor,'FontName',cjkFont);
        else
            % 一般情况: 显示最近对和距离
            pairStr = sprintf('pair(%d,%d) = %.1fmm', selfPair(1), selfPair(2), dist);
            pairColor = [0.3 0.3 0.5];
            text(ax,0.08,yP,pairStr,'FontSize',9,'FontWeight','bold','Color',pairColor,'FontName',cjkFont);
        end
    else
        text(ax,0.08,yP,'pair: --','FontSize',9,'Color',[0.5 0.5 0.5],'FontName',cjkFont);
    end
    
    % === 环境碰撞 (区域约束) ===
    yP=yP-0.035;
    if envCollFrame
        ec=[0.9 0.1 0.1]; ecStr='当前帧碰撞!';
    elseif envColl > 0
        ec=[0.9 0.5 0.0]; ecStr=sprintf('累计 %d 帧', envColl);
    else
        ec=[0 0.6 0.2]; ecStr='无碰撞';
    end
    text(ax,0.08,yP,sprintf('Env: %s', ecStr),'FontSize',11,'FontWeight','bold','Color',ec,'FontName',cjkFont);
    
    % === 搬运状态 + 工具球信息 ===
    yP=yP-0.035;
    if carrying
        text(ax,0.08,yP,sprintf('搬运中 | 已放置: %d/%d', nPlaced, nBoxTotal),...
            'FontSize',10,'FontWeight','bold','Color',[0.8 0.5 0],'FontName',cjkFont);
        yP=yP-0.022;
        text(ax,0.08,yP,'工具球 idx=1 z=-400 r=120mm',...
            'FontSize',8,'FontWeight','bold','Color',[0.2 0.55 0.3],'FontName',cjkFont);
        yP=yP-0.018;
        text(ax,0.08,yP,'覆盖: z=-280~-520mm (箱底保护)',...
            'FontSize',7,'Color',[0.4 0.4 0.4],'FontName',cjkFont);
    else
        text(ax,0.08,yP,sprintf('空夹 | 已放置: %d/%d', nPlaced, nBoxTotal),...
            'FontSize',10,'FontWeight','bold','Color',[0.4 0.4 0.4],'FontName',cjkFont);
    end
    
    % === TCP 位置 ===
    yP=yP-0.035;
    text(ax,0.05,yP,'TCP (mm)','FontSize',11,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.022;
    labels={'X','Y','Z'}; colors_xyz={[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8]};
    for ci=1:3
        text(ax,0.08,yP,sprintf('%s: %+8.1f',labels{ci},tcp(ci)*1000),...
            'FontSize',10,'FontWeight','bold','Color',colors_xyz{ci},'FontName',cjkFont);
        yP=yP-0.020;
    end
    
    % === 关节角度条 ===
    yP=yP-0.005;
    text(ax,0.05,yP,'Joint (deg)','FontSize',10,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.020;
    for ji=1:6
        barFrac=abs(q_deg(ji))/360; barW=max(0.01,barFrac*0.45);
        if q_deg(ji)>=0, bC=[0.3 0.6 0.9]; else, bC=[0.9 0.5 0.2]; end
        rectangle(ax,'Position',[0.22 yP-0.005 0.45 0.010],'FaceColor',[0.93 0.93 0.93],'EdgeColor','none');
        rectangle(ax,'Position',[0.22 yP-0.005 barW 0.010],'FaceColor',bC,'EdgeColor','none','Curvature',0.3);
        text(ax,0.06,yP,sprintf('J%d',ji),'FontSize',8,'FontWeight','bold','Color',[0.3 0.3 0.3],'FontName',cjkFont);
        text(ax,0.72,yP,sprintf('%+6.1f',q_deg(ji)),'FontSize',8,'FontWeight','bold','Color',bC,'FontName',cjkFont);
        yP=yP-0.017;
    end
    
    % === 时间 ===
    yP=yP-0.008;
    text(ax,0.08,yP,sprintf('Time: %.3f s', time_s),...
        'FontSize',10,'FontWeight','bold','Color',[0.2 0.2 0.2],'FontName',cjkFont);
end
