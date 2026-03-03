function drawInfoPanel_v15(ax, taskIdx, nTotal, q_deg, vel, tcp, dist, time_s, cjkFont, dColor, soActive, carrying, nPlaced, nBoxTotal, envColl)
%DRAWINFOPANEL_V15 绘制实时信息面板 (v15版, 含环境碰撞+搬运状态)
    rectangle(ax,'Position',[0 0 1 1],'FaceColor',[0.97 0.97 0.99],...
        'EdgeColor',[0.5 0.5 0.7],'LineWidth',2,'Curvature',0.02);
    yP = 0.96;
    text(ax,0.5,yP,'v15 ENV COLLISION','FontSize',14,'FontWeight','bold',...
        'HorizontalAlignment','center','Color',[0.1 0.1 0.3],'FontName',cjkFont);
    yP=yP-0.035;
    if soActive, srcStr = 'libHRCInterface.so (实时)'; else, srcStr = 'C++ (预计算)'; end
    text(ax,0.5,yP,srcStr,'FontSize',9,'FontWeight','bold','HorizontalAlignment','center',...
        'Color',[0.3 0.3 0.5],'FontName',cjkFont);
    
    yP=yP-0.05;
    progW = max(0.01, 0.86*(taskIdx/nTotal));
    rectangle(ax,'Position',[0.05 yP-0.03 0.90 0.04],'FaceColor',[0.15 0.35 0.65],'EdgeColor','none','Curvature',0.3);
    rectangle(ax,'Position',[0.07 yP-0.025 progW 0.03],'FaceColor',[0.3 0.75 0.45],'EdgeColor','none','Curvature',0.3);
    text(ax,0.5,yP-0.01,sprintf('Task %d / %d  [FIFO]',taskIdx,nTotal),...
        'FontSize',12,'FontWeight','bold','HorizontalAlignment','center','Color','w','FontName',cjkFont);
    
    % Self-distance
    yP=yP-0.06;
    rectangle(ax,'Position',[0.05 yP-0.035 0.90 0.05],'FaceColor',[1 1 1],...
        'EdgeColor',dColor,'LineWidth',3,'Curvature',0.2);
    text(ax,0.5,yP-0.01,sprintf('Self: %.1f mm', dist),...
        'FontSize',14,'FontWeight','bold','HorizontalAlignment','center','Color',dColor,'FontName',cjkFont);
    
    % Env collision
    yP=yP-0.06;
    if envColl==0, ec=[0 0.6 0.2]; ecStr='CLEAR'; else, ec=[0.9 0.1 0.1]; ecStr=sprintf('%d',envColl); end
    text(ax,0.08,yP,sprintf('Env: %s', ecStr),'FontSize',12,'FontWeight','bold','Color',ec,'FontName',cjkFont);
    
    % Carrying / Placed status
    yP=yP-0.035;
    if carrying
        text(ax,0.08,yP,sprintf('搬运中 | 已放置: %d/%d', nPlaced, nBoxTotal),...
            'FontSize',10,'FontWeight','bold','Color',[0.8 0.5 0],'FontName',cjkFont);
    else
        text(ax,0.08,yP,sprintf('空夹 | 已放置: %d/%d', nPlaced, nBoxTotal),...
            'FontSize',10,'FontWeight','bold','Color',[0.4 0.4 0.4],'FontName',cjkFont);
    end
    
    % TCP
    yP=yP-0.04;
    text(ax,0.05,yP,'TCP (mm)','FontSize',11,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.025;
    labels={'X','Y','Z'}; colors_xyz={[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8]};
    for ci=1:3
        text(ax,0.08,yP,sprintf('%s: %+8.1f',labels{ci},tcp(ci)*1000),...
            'FontSize',11,'FontWeight','bold','Color',colors_xyz{ci},'FontName',cjkFont);
        yP=yP-0.022;
    end
    
    % Joint bars
    yP=yP-0.01;
    text(ax,0.05,yP,'Joint (deg)','FontSize',11,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.022;
    for ji=1:6
        barFrac=abs(q_deg(ji))/360; barW=max(0.01,barFrac*0.45);
        if q_deg(ji)>=0, bC=[0.3 0.6 0.9]; else, bC=[0.9 0.5 0.2]; end
        rectangle(ax,'Position',[0.22 yP-0.006 0.45 0.012],'FaceColor',[0.93 0.93 0.93],'EdgeColor','none');
        rectangle(ax,'Position',[0.22 yP-0.006 barW 0.012],'FaceColor',bC,'EdgeColor','none','Curvature',0.3);
        text(ax,0.06,yP,sprintf('J%d',ji),'FontSize',8,'FontWeight','bold','Color',[0.3 0.3 0.3],'FontName',cjkFont);
        text(ax,0.72,yP,sprintf('%+6.1f',q_deg(ji)),'FontSize',9,'FontWeight','bold','Color',bC,'FontName',cjkFont);
        yP=yP-0.019;
    end
    
    yP=yP-0.01;
    text(ax,0.08,yP,sprintf('Time: %.3f s', time_s),...
        'FontSize',10,'FontWeight','bold','Color',[0.2 0.2 0.2],'FontName',cjkFont);
end
