%% testS50_Palletizing_v9.m - HR_S50-2000 码垛工作站 v9.0 (性能优化版)
%  v9.0 改进:
%    - 中文字体: Noto Sans CJK SC (解决乱码)
%    - 高清输出: print -dpng -r200 (提升清晰度)
%    - 性能优化: 降低mesh面数, 预计算FK缓存, 精简绘图对象
%    - 碰撞检测: 解析线段-AABB距离算法 (替代采样法)
%    - 仪表盘: 英文标识符+中文标签, 避免等宽字体乱码
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

close all; clear all; clc;
addpath('collisionVisual'); addpath(genpath('collisionVisual'));

%% ====================== 字体与渲染设置 ======================
% 检测中文字体
CJK_FONT = 'Noto Sans CJK SC';
allFonts = listfonts();
if ~any(strcmp(allFonts, CJK_FONT))
    candidates = {'Noto Serif CJK SC','Noto Sans CJK SC Medium','SimHei','WenQuanYi Micro Hei','Arial Unicode MS'};
    CJK_FONT = '';
    for ci = 1:length(candidates)
        if any(strcmp(allFonts, candidates{ci}))
            CJK_FONT = candidates{ci};
            break;
        end
    end
    if isempty(CJK_FONT)
        CJK_FONT = get(0, 'DefaultAxesFontName');
        warning('未找到CJK字体, 使用默认字体: %s', CJK_FONT);
    end
end
MONO_FONT = 'Noto Sans Mono CJK SC';
if ~any(strcmp(allFonts, MONO_FONT))
    MONO_FONT = CJK_FONT;
end
fprintf('Font: %s / %s\n', CJK_FONT, MONO_FONT);

% 全局默认字体
set(0, 'DefaultAxesFontName', CJK_FONT);
set(0, 'DefaultTextFontName', CJK_FONT);

% Mesh分辨率 (减少面数提升性能)
SPHERE_N  = 8;   % sphere(N): v8用12, 降低到8
CYL_N     = 8;   % cylinder(r, N): v8用10-12
DPI       = 200; % 输出DPI

%% ====================== 环境设置 ======================
isHeadless = ~usejava('desktop');
outputDir = './pic/S50_palletizing_v9';
if isHeadless, set(0, 'DefaultFigureVisible', 'off'); end
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

%% ====================== 场景几何参数 (m) ======================
cab.widthX  = 0.55;  cab.depthY  = 0.65;  cab.heightZ = 0.80;
cab.color   = [0.95, 0.95, 0.93];

frame.widthX = 1.20;  frame.depthY = 1.00;  frame.height = 2.00;
frame.tubeR  = 0.030; frame.color  = [0.25, 0.55, 0.85];

pallet.widthX = 1.00;  pallet.depthY = 0.80;
pallet.heightZ = 0.55; pallet.color  = [0.20, 0.45, 0.80];

conv.lengthY  = 2.00;  conv.widthX = 0.55;  conv.heightZ = 0.75;
conv.beltH    = 0.035; conv.rollerR = 0.030; conv.nRollers = 12;
conv.color    = [0.30, 0.30, 0.32];

box.lx = 0.40; box.wy = 0.30; box.hz = 0.25;
box.color = [0.65, 0.45, 0.25];
nBoxes = 3;

%% ====================== 碰撞安全参数 ======================
COLL_THRESH_DANGER  = 0.00;
COLL_THRESH_WARNING = 0.05;
COLL_THRESH_CAUTION = 0.10;
GROUND_Z = 0.0;

%% ====================== 布局 ======================
baseX = 0.0; baseY = 0.0; baseZ = cab.heightZ;

frameGap = 0.40;
frame.cx = 0.0;
frame.cy = cab.depthY/2 + frameGap + frame.depthY/2;

convGap = 0.40;
conv.cx = cab.widthX/2 + convGap + conv.widthX/2;
conv.cy = -0.30;

convSurfZ = conv.heightZ + conv.rollerR + conv.beltH;
convBoxTopZ = convSurfZ + box.hz;
palletSurfZ = pallet.heightZ;

fprintf([char(9556) repmat(char(9552),1,70) char(9559) '\n']);
fprintf([char(9553) '  HR_S50-2000 v9.0 -- Optimized 3-Layer Collision (3-box palletizing)    ' char(9553) '\n']);
fprintf([char(9562) repmat(char(9552),1,70) char(9565) '\n\n']);

fprintf('Scene Layout:\n');
fprintf('  Cabinet: (%.2f,%.2f) %.2f x %.2f x %.2fm\n', baseX,baseY,cab.widthX,cab.depthY,cab.heightZ);
fprintf('  Frame:   (%.2f,%.2f) open=-Y  pallet=%.2fm\n', frame.cx,frame.cy,pallet.heightZ);
fprintf('  Conv:    (%.2f,%.2f) belt=%.3fm  boxTop=%.3fm\n', conv.cx, conv.cy, convSurfZ, convBoxTopZ);
fprintf('  Base Z = %.2fm\n\n', baseZ);

%% ====================== 箱子位置 ======================
convBoxY = zeros(1, nBoxes);
yStart = conv.cy - 0.15;
for bi = 1:nBoxes
    convBoxY(bi) = yStart - (bi-1)*0.45;
end
convBoxX = conv.cx;

placePos = zeros(nBoxes, 3);
placePos(1,:) = [frame.cx - box.lx/2, frame.cy, palletSurfZ + box.hz];
placePos(2,:) = [frame.cx + box.lx/2, frame.cy, palletSurfZ + box.hz];
placePos(3,:) = [frame.cx,            frame.cy, palletSurfZ + 2*box.hz];

for bi=1:nBoxes
    fprintf('  Box%d pick=[%.3f,%.3f,%.3f] place=[%.3f,%.3f,%.3f]\n', ...
        bi, convBoxX, convBoxY(bi), convBoxTopZ, placePos(bi,:));
end
fprintf('\n');

%% ====================== 碰撞模型 ======================
params = readCollisionModelJson("./model/collideConfig/S50_collision.json");
params_tool = readToolCollisionJson("./model/collideConfig/nonetool_collision.json");

global d1 d2 d3 d4 d5 d6 a2 a3 T6T;
d1=params.DH.d1; d2=params.DH.d2; d3=params.DH.d3;
d4=params.DH.d4; d5=params.DH.d5; d6=params.DH.d6;
a2=-params.DH.a2; a3=-params.DH.a3; T6T=eye(4);

Tb = eye(4); Tb(1,4)=baseX; Tb(2,4)=baseY; Tb(3,4)=baseZ;

%% ====================== 环境碰撞模型 ======================
envObjs = buildEnvironmentModel(cab, frame, pallet, conv, baseX, baseY, GROUND_Z);
fprintf('Environment: %d collision bodies\n', length(envObjs));
for ei = 1:length(envObjs)
    fprintf('  [%2d] %-10s [%.2f,%.2f,%.2f]~[%.2f,%.2f,%.2f]\n', ...
        ei, envObjs(ei).name, envObjs(ei).aabbMin, envObjs(ei).aabbMax);
end
fprintf('\n');

%% ====================== IK ======================
fprintf('IK solving...\n');
pickIK = struct('q1',{},'q2',{},'q3',{},'q4',{},'err',{});
for bi = 1:nBoxes
    dx = convBoxX - baseX; dy = convBoxY(bi) - baseY;
    q1 = atan2(-dy, -dx);
    tr = sqrt(dx^2+dy^2); tz = convBoxTopZ - baseZ;
    [q2,q3,q4,err] = searchIK_ext(q1, tr, tz);
    pickIK(bi).q1=q1; pickIK(bi).q2=q2; pickIK(bi).q3=q3; pickIK(bi).q4=q4; pickIK(bi).err=err;
    fprintf('  Pick%d: J1=%+.1f err=%.1fmm\n', bi, rad2deg(q1), err*1000);
end

placeIK = struct('q1',{},'q2',{},'q3',{},'q4',{},'err',{});
for bi = 1:nBoxes
    dx = placePos(bi,1)-baseX; dy = placePos(bi,2)-baseY;
    q1 = atan2(-dy,-dx);
    tr = sqrt(dx^2+dy^2); tz = placePos(bi,3) - baseZ;
    [q2,q3,q4,err] = searchIK_ext(q1, tr, tz);
    placeIK(bi).q1=q1; placeIK(bi).q2=q2; placeIK(bi).q3=q3; placeIK(bi).q4=q4; placeIK(bi).err=err;
    fprintf('  Place%d: J1=%+.1f err=%.1fmm\n', bi, rad2deg(q1), err*1000);
end
fprintf('\n');

%% ====================== 姿态序列 ======================
allPoses = {};
idx_f = 0;
q_home = [pickIK(1).q1, -pi/3, pi/4, 0, -pi/3, 0];

idx_f=idx_f+1;
allPoses{idx_f} = struct('name','Initial','label','Initial Standby', ...
    'q',q_home,'chk','none','boxId',0,'placed',0, ...
    'convRemain',1:nBoxes,'carrying',false);

for bi = 1:nBoxes
    pk = pickIK(bi); pl = placeIK(bi);
    q_grasp   = [pk.q1, pk.q2, pk.q3, pk.q4, -pi/2, 0];
    q_release = [pl.q1, pl.q2, pl.q3, pl.q4, -pi/2, 0];
    q_hov_pk  = q_grasp;
    q_hov_pk(2) = q_hov_pk(2)+deg2rad(8); q_hov_pk(3) = q_hov_pk(3)-deg2rad(6);
    q_hov_pl  = q_release;
    q_hov_pl(2) = q_hov_pl(2)+deg2rad(8); q_hov_pl(3) = q_hov_pl(3)-deg2rad(6);
    
    crB = bi:nBoxes; crA = (bi+1):nBoxes;
    pB = bi-1; pA = bi;
    
    idx_f=idx_f+1;
    allPoses{idx_f} = struct('name',sprintf('HoverPick%d',bi),'label',sprintf('Hover Pick #%d',bi), ...
        'q',q_hov_pk,'chk','none','boxId',bi,'placed',pB,'convRemain',crB,'carrying',false);
    idx_f=idx_f+1;
    allPoses{idx_f} = struct('name',sprintf('Pick%d',bi),'label',sprintf('Pick Box #%d',bi), ...
        'q',q_grasp,'chk','pick','boxId',bi,'placed',pB,'convRemain',crB,'carrying',false);
    idx_f=idx_f+1;
    allPoses{idx_f} = struct('name',sprintf('Lift%d',bi),'label',sprintf('Lift Box #%d',bi), ...
        'q',q_hov_pk,'chk','none','boxId',bi,'placed',pB,'convRemain',crA,'carrying',true);
    idx_f=idx_f+1;
    allPoses{idx_f} = struct('name',sprintf('Turn%d',bi),'label',sprintf('Turn to Place #%d',bi), ...
        'q',q_hov_pl,'chk','none','boxId',bi,'placed',pB,'convRemain',crA,'carrying',true);
    idx_f=idx_f+1;
    allPoses{idx_f} = struct('name',sprintf('Place%d',bi),'label',sprintf('Place Box #%d',bi), ...
        'q',q_release,'chk','place','boxId',bi,'placed',pB,'convRemain',crA,'carrying',true);
    idx_f=idx_f+1;
    allPoses{idx_f} = struct('name',sprintf('Retract%d',bi),'label',sprintf('Retract from #%d',bi), ...
        'q',q_hov_pl,'chk','none','boxId',bi,'placed',pA,'convRemain',crA,'carrying',false);
end

idx_f=idx_f+1;
allPoses{idx_f} = struct('name','Done','label','Task Complete', ...
    'q',q_home,'chk','none','boxId',0,'placed',nBoxes, ...
    'convRemain',[],'carrying',false);

N = length(allPoses);
fprintf('Total frames: %d\n\n', N);

%% ====================== 预计算全帧FK+碰撞 ======================
fprintf('Pre-computing FK & collision for all %d frames...\n', N);
tic;

% 预分配结构体
precomp = struct();
for idx = 1:N
    q = allPoses{idx}.q;
    [T00,T01,T02,T03,T04,T05,T0T] = FK_SSerial(q);
    Tf = {Tb*T00, Tb*T01, Tb*T02, Tb*T03, Tb*T04, Tb*T05, Tb*T0T};
    
    % 碰撞体端点 (不绘图)
    outS = computeCollisionPoints(Tf, params);
    
    % 三层碰撞检测
    [sc, selfInfo, selfMinDist, selfPairDists] = selfCollCheck_v9(outS, params);
    [ec, envInfo, envMinDist, envDetailDists] = envCollCheck_v9(outS, params, envObjs);
    
    ep = Tf{7}(1:3,4);
    ez = T0T(1:3,3);
    dev = acosd(max(-1,min(1,dot(ez,[0;0;-1]))));
    tcpOnGround = ep(3) < GROUND_Z + 0.02;
    
    overallMin = min(selfMinDist, envMinDist);
    isColl = sc || ec || tcpOnGround;
    isWarn = ~isColl && overallMin < COLL_THRESH_WARNING;
    isCaut = ~isColl && ~isWarn && overallMin < COLL_THRESH_CAUTION;
    
    precomp(idx).Tf = Tf;
    precomp(idx).outS = outS;
    precomp(idx).ep = ep;
    precomp(idx).dev = dev;
    precomp(idx).sc = sc;  precomp(idx).selfInfo = selfInfo;
    precomp(idx).selfMinDist = selfMinDist; precomp(idx).selfPairDists = selfPairDists;
    precomp(idx).ec = ec;  precomp(idx).envInfo = envInfo;
    precomp(idx).envMinDist = envMinDist;   precomp(idx).envDetailDists = envDetailDists;
    precomp(idx).tcpOnGround = tcpOnGround;
    precomp(idx).overallMin = overallMin;
    precomp(idx).isColl = isColl;
    precomp(idx).isWarn = isWarn;
    precomp(idx).isCaut = isCaut;
end
fprintf('Pre-computation done in %.1fs\n\n', toc);

%% ====================== 预计算热力图数据 ======================
heatData = zeros(5, N);
for idx = 1:N
    outS2 = precomp(idx).outS;
    linkSegs = getLinkSegments(outS2, params);
    for li = 1:5
        minD = inf;
        for oi = 1:length(envObjs)
            d = segAABBDist_analytic(linkSegs(li).p1, linkSegs(li).p2, linkSegs(li).r, ...
                envObjs(oi).aabbMin, envObjs(oi).aabbMax);
            if d < minD, minD = d; end
        end
        heatData(li, idx) = minD;
    end
end

%% ====================== 主渲染循环 (v9.0 优化) ======================
res = cell(N,1);
collisionLog = zeros(N, 5);  % [selfDist envDist minDist isColl isWarn]

for idx = 1:N
    p = allPoses{idx};
    pc = precomp(idx);
    
    fprintf('---[%d/%d] %s  q=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f] deg\n', ...
        idx, N, p.label, rad2deg(p.q));
    fprintf('   TCP=[%.3f,%.3f,%.3f]m  dev=%.1f deg\n', pc.ep, pc.dev);
    
    % TCP验证
    tcpOK = true;
    if strcmp(p.chk,'pick')
        tgt = [convBoxX, convBoxY(p.boxId), convBoxTopZ];
        gXY = sqrt((pc.ep(1)-tgt(1))^2 + (pc.ep(2)-tgt(2))^2);
        gZ = abs(pc.ep(3)-tgt(3));
        tcpOK = gZ<0.03 && gXY<0.30;
        fprintf('   Pick#%d: errXY=%.1fmm Z=%.1fmm %s\n', p.boxId, gXY*1000, gZ*1000, iff(tcpOK,'OK','NG'));
    elseif strcmp(p.chk,'place')
        tgt = placePos(p.boxId,:);
        gXY = sqrt((pc.ep(1)-tgt(1))^2 + (pc.ep(2)-tgt(2))^2);
        gZ = abs(pc.ep(3)-tgt(3));
        tcpOK = gZ<0.04 && gXY<0.30;
        fprintf('   Place#%d: errXY=%.1fmm Z=%.1fmm %s\n', p.boxId, gXY*1000, gZ*1000, iff(tcpOK,'OK','NG'));
    end
    
    % 碰撞状态日志
    if pc.isColl
        fprintf('   [COLLISION] ');
        if pc.sc, fprintf('Self: %s ', pc.selfInfo); end
        if pc.ec, fprintf('Env: %s ', pc.envInfo); end
        if pc.tcpOnGround, fprintf('TCP-Ground '); end
        fprintf('\n');
        tc = [0.85 0 0];
    elseif pc.isWarn
        fprintf('   [WARNING] minDist=%.1fmm (self=%.1f env=%.1f)\n', ...
            pc.overallMin*1000, pc.selfMinDist*1000, pc.envMinDist*1000);
        tc = [0.85 0.55 0];
    elseif pc.isCaut
        fprintf('   [CAUTION] minDist=%.1fmm\n', pc.overallMin*1000);
        tc = [0.70 0.55 0.10];
    else
        fprintf('   [SAFE] minDist=%.1fmm\n', pc.overallMin*1000);
        tc = [0 0.55 0];
    end
    
    collisionLog(idx,:) = [pc.selfMinDist, pc.envMinDist, pc.overallMin, pc.isColl, pc.isWarn];
    
    % ===== 渲染 =====
    fig = figure('Position', [50 50 1600 950], 'Color', 'w', ...
        'Renderer', 'opengl');
    
    % --- 左: 3D场景 ---
    ax_main = axes('Position', [0.02 0.05 0.66 0.88]); hold on;
    global alpha; alpha = 0.5;
    
    drawGround_v9(-1.5, 2.0, -2.0, 3.0);
    drawCabinet_v9(cab, baseX, baseY, CJK_FONT);
    drawFrame_v9(frame, CYL_N);
    drawPallet_v9(pallet, frame, CJK_FONT);
    drawConveyor_v9(conv, CYL_N);
    
    % 传送带上剩余箱子
    bz_center = convSurfZ + box.hz/2;
    for ci = 1:length(p.convRemain)
        bi2 = p.convRemain(ci);
        drawBox_v9([convBoxX, convBoxY(bi2), bz_center], box);
    end
    % 蓝框内已放好的箱子
    for pi2 = 1:p.placed
        drawBox_v9([placePos(pi2,1), placePos(pi2,2), placePos(pi2,3)-box.hz/2], box);
    end
    
    % 机器人
    Tf = pc.Tf;
    for fi=1:7, plotframe(Tf{fi}, 0.08, true); end
    outS = plotSelfCollisonModel(Tf, params, params_tool);
    
    % 携带箱子
    if p.carrying
        drawBox_v9([pc.ep(1), pc.ep(2), pc.ep(3)-0.01], box);
    end
    
    % 碰撞热力图渲染 (半透明轮廓)
    drawCollisionHeatmap_v9(outS, params, pc.selfPairDists, pc.envDetailDists, ...
        COLL_THRESH_DANGER, COLL_THRESH_WARNING, COLL_THRESH_CAUTION, SPHERE_N, CYL_N);
    
    % 最小距离连线
    drawMinDistLine(pc.selfPairDists, COLL_THRESH_CAUTION, 'r', CJK_FONT);
    drawMinDistLine(pc.envDetailDists, COLL_THRESH_CAUTION, [0.6 0 0.6], CJK_FONT);
    
    % 取放标记
    if strcmp(p.chk,'pick')
        plot3(convBoxX, convBoxY(p.boxId), convBoxTopZ, 'rv', 'MarkerSize',12, 'LineWidth',2);
        plot3(pc.ep(1), pc.ep(2), pc.ep(3), 'g^', 'MarkerSize',12, 'LineWidth',2);
    elseif strcmp(p.chk,'place')
        plot3(placePos(p.boxId,1), placePos(p.boxId,2), placePos(p.boxId,3), 'rv','MarkerSize',12,'LineWidth',2);
        plot3(pc.ep(1), pc.ep(2), pc.ep(3), 'g^', 'MarkerSize',12, 'LineWidth',2);
    end
    
    % 传送方向箭头
    arrZ = convSurfZ + conv.beltH + 0.08;
    plot3([conv.cx conv.cx], [conv.cy-conv.lengthY/2+0.15, conv.cy+conv.lengthY/2-0.15], ...
        [arrZ arrZ], 'm-', 'LineWidth', 1.5);
    
    % 箱号标注
    for ci = 1:length(p.convRemain)
        bi2 = p.convRemain(ci);
        text(convBoxX, convBoxY(bi2), convSurfZ+box.hz+0.06, sprintf('#%d',bi2), ...
            'FontSize',9, 'FontWeight','bold', 'FontName',CJK_FONT, ...
            'HorizontalAlignment','center', 'Color',[.8 .3 0]);
    end
    for pi2 = 1:p.placed
        text(placePos(pi2,1), placePos(pi2,2), placePos(pi2,3)+0.06, sprintf('#%d',pi2), ...
            'FontSize',9, 'FontWeight','bold', 'FontName',CJK_FONT, ...
            'HorizontalAlignment','center', 'Color',[.0 .4 .7]);
    end
    
    % 标题 - 使用CJK字体
    title(sprintf('HR S50-2000 v9.0: [%d/%d] %s', idx,N,p.label), ...
        'FontSize', 14, 'Color', tc, 'FontWeight', 'bold', 'FontName', CJK_FONT);
    xlabel('X (m)', 'FontName', CJK_FONT, 'FontSize', 11);
    ylabel('Y (m)', 'FontName', CJK_FONT, 'FontSize', 11);
    zlabel('Z (m)', 'FontName', CJK_FONT, 'FontSize', 11);
    set(gca, 'FontName', CJK_FONT, 'FontSize', 10);
    axis equal; grid on;
    xlim([-1.5 2.0]); ylim([-2.0 3.0]); zlim([0 2.5]);
    view(-45, 25);
    camlight('headlight'); lighting gouraud;
    
    % --- 右: 仪表盘 ---
    ax_dash = axes('Position', [0.70 0.05 0.28 0.88]); hold on;
    axis off;
    drawDashboard_v9(ax_dash, idx, N, p.label, pc, ...
        COLL_THRESH_WARNING, COLL_THRESH_CAUTION, CJK_FONT, MONO_FONT);
    
    drawnow limitrate;
    
    % 保存结果
    res{idx} = struct('name',p.name, 'label',p.label, 'ep',pc.ep, 'dev',pc.dev, ...
        'sc',pc.sc, 'ec',pc.ec, 'sd',pc.selfMinDist, 'ed',pc.envMinDist, ...
        'tcpOK',tcpOK, 'chk',p.chk, 'boxId',p.boxId, ...
        'overallMinDist',pc.overallMin, 'isCollision',pc.isColl);
    
    if isHeadless
        fn = sprintf('%s/pose_%02d.png', outputDir, idx);
        print(fig, fn, '-dpng', sprintf('-r%d', DPI));
        close(fig);
        fprintf('   >> %s\n', fn);
    end
end

%% ====================== 汇总 ======================
fprintf('\n');
fprintf('+----+------------------+------+----------+----------+----------+------+\n');
fprintf('| Fr | Action           | Stat | Self(mm) | Env(mm)  | Min(mm)  | Note |\n');
fprintf('+----+------------------+------+----------+----------+----------+------+\n');
cc=0; wc=0; tcpAll=true;
for i=1:N
    r=res{i};
    if r.isCollision, s='COLL'; cc=cc+1;
    elseif r.overallMinDist<COLL_THRESH_WARNING, s='WARN'; wc=wc+1;
    elseif r.overallMinDist<COLL_THRESH_CAUTION, s='CAUT';
    else, s='SAFE';
    end
    if ~r.tcpOK, tcpAll=false; end
    extra='';
    if strcmp(r.chk,'pick'),  extra=sprintf('Pk#%d',r.boxId); end
    if strcmp(r.chk,'place'), extra=sprintf('Pl#%d',r.boxId); end
    fprintf('| %2d | %-16s | %-4s | %7.1f  | %7.1f  | %7.1f  | %-4s |\n', ...
        i, r.label, s, r.sd*1000, r.ed*1000, r.overallMinDist*1000, extra);
end
fprintf('+----+------------------+------+----------+----------+----------+------+\n');
fprintf('  Collisions=%d  Warnings=%d  Frames=%d  TCP=%s  Boxes=%d\n', ...
    cc, wc, N, iff(tcpAll,'OK','NG'), nBoxes);
fprintf('  Detection: [1]Self(5-body,9-pair) [2]Env(%d objects) [3]TCP-zone\n\n', length(envObjs));

%% ====================== 趋势图 ======================
fig_trend = figure('Position', [100 100 1400 550], 'Color', 'w');

% 左: 距离趋势
subplot(1,2,1);
frames = 1:N;
hold on;
fill([frames fliplr(frames)], [zeros(1,N) ones(1,N)*COLL_THRESH_DANGER], ...
    [1 0.85 0.85], 'EdgeColor','none', 'FaceAlpha', 0.3);
fill([frames fliplr(frames)], [ones(1,N)*COLL_THRESH_DANGER ones(1,N)*COLL_THRESH_WARNING], ...
    [1 1 0.8], 'EdgeColor','none', 'FaceAlpha', 0.3);
fill([frames fliplr(frames)], [ones(1,N)*COLL_THRESH_WARNING ones(1,N)*COLL_THRESH_CAUTION], ...
    [1 0.95 0.8], 'EdgeColor','none', 'FaceAlpha', 0.3);

plot(frames, collisionLog(:,1)', 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
plot(frames, collisionLog(:,2)', 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
plot(frames, collisionLog(:,3)', 'k-d', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor','y');

yline(COLL_THRESH_DANGER, 'r--', 'Collision', 'LineWidth', 1.5, 'FontName', CJK_FONT);
yline(COLL_THRESH_WARNING, 'Color', [0.8 0.6 0], 'LineStyle', '--', 'Label', 'Warning', 'FontName', CJK_FONT);
yline(COLL_THRESH_CAUTION, 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'Label', 'Caution', 'FontName', CJK_FONT);

legend({'Collision Zone','Warning Zone','Caution Zone','Self-Collision','Env-Collision','Overall Min'}, ...
    'Location', 'best', 'FontName', CJK_FONT, 'FontSize', 9);
xlabel('Frame', 'FontName', CJK_FONT, 'FontSize', 11);
ylabel('Min Distance (m)', 'FontName', CJK_FONT, 'FontSize', 11);
title('Collision Distance Trend', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', CJK_FONT);
grid on; xlim([0.5 N+0.5]);
set(gca, 'FontName', CJK_FONT, 'FontSize', 10);

% 右: 热力图
subplot(1,2,2);
linkLabels = {'Base','LowerArm','Elbow','UpperArm','Wrist'};
imagesc(heatData);
colormap(gca, customHeatmap());
cb = colorbar; ylabel(cb, 'Distance (m)', 'FontName', CJK_FONT, 'FontSize', 10);
caxis([0 0.3]);
set(gca, 'YTick', 1:5, 'YTickLabel', linkLabels, 'FontName', CJK_FONT);
set(gca, 'XTick', 1:N);
xlabel('Frame', 'FontName', CJK_FONT, 'FontSize', 11);
ylabel('Link', 'FontName', CJK_FONT, 'FontSize', 11);
title('Link-Env Distance Heatmap (m)', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', CJK_FONT);
set(gca, 'FontName', CJK_FONT, 'FontSize', 10);

if isHeadless
    print(fig_trend, sprintf('%s/collision_trend.png', outputDir), '-dpng', sprintf('-r%d', DPI));
    close(fig_trend);
    fprintf('Trend chart saved\n');
end

%% ====================== GIF ======================
if isHeadless
    gifFile = sprintf('%s/palletizing_v9_anim.gif', outputDir);
    fprintf('Generating GIF: %s\n', gifFile);
    for idx = 1:N
        fn = sprintf('%s/pose_%02d.png', outputDir, idx);
        if ~exist(fn,'file'), continue; end
        img = imread(fn);
        [A,map] = rgb2ind(img, 256);
        delay = 0.8;
        if idx==1 || idx==N, delay=1.5; end
        if contains(allPoses{idx}.name,'Pick') || contains(allPoses{idx}.name,'Place')
            delay = 1.2;
        end
        if idx == 1
            imwrite(A, map, gifFile, 'gif', 'LoopCount', 0, 'DelayTime', delay);
        else
            imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
        end
    end
    fprintf('GIF done (%d frames)\n', N);
end

fprintf('\nv9.0 palletizing simulation complete!\n');

%% ========================================================================
%%                          v9.0 Functions
%% ========================================================================

%% ---- 环境碰撞模型 ----
function objs = buildEnvironmentModel(cab, frame, pallet, conv, bx, by, groundZ)
    objs = struct('name',{}, 'aabbMin',{}, 'aabbMax',{}, 'type',{});
    
    objs(end+1).name = 'Ground';
    objs(end).aabbMin = [-2, -3, groundZ-0.02]; objs(end).aabbMax = [3, 4, groundZ]; objs(end).type = 'ground';
    
    objs(end+1).name = 'Cabinet';
    objs(end).aabbMin = [bx-cab.widthX/2, by-cab.depthY/2, 0];
    objs(end).aabbMax = [bx+cab.widthX/2, by+cab.depthY/2, cab.heightZ]; objs(end).type = 'box';
    
    wx = frame.widthX; dy = frame.depthY; cx = frame.cx; cy = frame.cy;
    corners = [cx-wx/2 cy-dy/2; cx+wx/2 cy-dy/2; cx+wx/2 cy+dy/2; cx-wx/2 cy+dy/2];
    pillarNames = {'Pillar-LF','Pillar-RF','Pillar-RB','Pillar-LB'};
    for i = 1:4
        objs(end+1).name = pillarNames{i};
        objs(end).aabbMin = [corners(i,1)-frame.tubeR, corners(i,2)-frame.tubeR, 0];
        objs(end).aabbMax = [corners(i,1)+frame.tubeR, corners(i,2)+frame.tubeR, frame.height];
        objs(end).type = 'pillar';
    end
    
    beamNames = {'Beam+X','Beam+Y','Beam-X'};
    beamEdges = {[2,3], [3,4], [4,1]};
    for bi = 1:3
        e = beamEdges{bi};
        objs(end+1).name = beamNames{bi};
        x1 = min(corners(e(1),1), corners(e(2),1)); x2 = max(corners(e(1),1), corners(e(2),1));
        y1 = min(corners(e(1),2), corners(e(2),2)); y2 = max(corners(e(1),2), corners(e(2),2));
        objs(end).aabbMin = [x1-frame.tubeR*0.8, y1-frame.tubeR*0.8, 0];
        objs(end).aabbMax = [x2+frame.tubeR*0.8, y2+frame.tubeR*0.8, frame.height];
        objs(end).type = 'beam';
    end
    
    objs(end+1).name = 'Pallet';
    objs(end).aabbMin = [frame.cx-pallet.widthX/2, frame.cy-pallet.depthY/2, 0];
    objs(end).aabbMax = [frame.cx+pallet.widthX/2, frame.cy+pallet.depthY/2, pallet.heightZ];
    objs(end).type = 'box';
    
    objs(end+1).name = 'Conveyor';
    objs(end).aabbMin = [conv.cx-conv.widthX/2, conv.cy-conv.lengthY/2, 0];
    objs(end).aabbMax = [conv.cx+conv.widthX/2, conv.cy+conv.lengthY/2, conv.heightZ+conv.rollerR+conv.beltH];
    objs(end).type = 'box';
end

%% ---- 解析线段-AABB距离 (替代采样法) ----
function d = segAABBDist_analytic(p1, p2, radius, aabbMin, aabbMax)
    % 将线段离散为解析分段检测: 用投影法快速求最近
    p1 = p1(:); p2 = p2(:); aabbMin = aabbMin(:); aabbMax = aabbMax(:);
    seg = p2 - p1;
    L = norm(seg);
    
    if L < 1e-10
        cp = max(aabbMin, min(p1, aabbMax));
        d = norm(p1 - cp) - radius;
        return;
    end
    
    dir = seg / L;
    
    % 对AABB的每个面找线段最近点 (解析法)
    minDistSq = inf;
    bestPtSeg = p1;
    bestPtBox = p1;
    
    % 在6个面上分别求线段到面的最近点
    % 面法向: +-X, +-Y, +-Z
    faces = [1 -1; 1 1; 2 -1; 2 1; 3 -1; 3 1];  % [axis, sign]
    
    for fi = 1:6
        ax = faces(fi, 1);
        if faces(fi, 2) < 0
            faceVal = aabbMin(ax);
        else
            faceVal = aabbMax(ax);
        end
        
        % 线段上距离面最近的参数t
        if abs(dir(ax)) > 1e-10
            t = (faceVal - p1(ax)) / dir(ax);
        else
            t = 0.5;  % 平行于面
        end
        t = max(0, min(L, t));
        
        ptSeg = p1 + t * dir;
        ptBox = max(aabbMin, min(ptSeg, aabbMax));
        dSq = sum((ptSeg - ptBox).^2);
        if dSq < minDistSq
            minDistSq = dSq;
            bestPtSeg = ptSeg;
            bestPtBox = ptBox;
        end
    end
    
    % 也检查线段端点
    for ti = [0, L]
        ptSeg = p1 + ti * dir;
        ptBox = max(aabbMin, min(ptSeg, aabbMax));
        dSq = sum((ptSeg - ptBox).^2);
        if dSq < minDistSq
            minDistSq = dSq;
            bestPtSeg = ptSeg;
            bestPtBox = ptBox;
        end
    end
    
    % 检查AABB的12条边与线段的最近点对
    % AABB的8个顶点
    verts = [aabbMin(1) aabbMin(2) aabbMin(3);
             aabbMax(1) aabbMin(2) aabbMin(3);
             aabbMax(1) aabbMax(2) aabbMin(3);
             aabbMin(1) aabbMax(2) aabbMin(3);
             aabbMin(1) aabbMin(2) aabbMax(3);
             aabbMax(1) aabbMin(2) aabbMax(3);
             aabbMax(1) aabbMax(2) aabbMax(3);
             aabbMin(1) aabbMax(2) aabbMax(3)];
    
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    
    for ei = 1:12
        e1 = verts(edges(ei,1),:)';
        e2 = verts(edges(ei,2),:)';
        [dd, cp1, cp2] = segSegDist_v9(p1, p2, e1, e2);
        dSq = dd^2;
        if dSq < minDistSq
            minDistSq = dSq;
            bestPtSeg = cp1;
            bestPtBox = cp2;
        end
    end
    
    d = sqrt(minDistSq) - radius;
    
    % 检查线段是否穿透AABB
    % 采样几个中点确认
    for t = [0.25, 0.5, 0.75]
        pt = p1 + t*L*dir;
        if all(pt >= aabbMin) && all(pt <= aabbMax)
            d = -radius - 0.001;  % 穿透
            return;
        end
    end
end

function [d, cpLink, cpBox] = segAABBDistFull_v9(p1, p2, radius, aabbMin, aabbMax)
    p1 = p1(:); p2 = p2(:); aabbMin = aabbMin(:); aabbMax = aabbMax(:);
    seg = p2 - p1;
    L = norm(seg);
    
    if L < 1e-10
        cpBox = max(aabbMin, min(p1, aabbMax));
        d2 = norm(p1 - cpBox);
        d = d2 - radius;
        if d2 > 1e-8
            cpLink = p1 + (cpBox - p1)/d2 * radius;
        else
            cpLink = p1;
        end
        return;
    end
    
    dir = seg / L;
    minDistSq = inf;
    bestPtSeg = p1;
    bestPtBox = p1;
    
    % 面投影
    for ax = 1:3
        for fv = [aabbMin(ax), aabbMax(ax)]
            if abs(dir(ax)) > 1e-10
                t = (fv - p1(ax)) / dir(ax);
            else
                t = 0.5*L;
            end
            t = max(0, min(L, t));
            ptSeg = p1 + t*dir;
            ptBox = max(aabbMin, min(ptSeg, aabbMax));
            dSq = sum((ptSeg-ptBox).^2);
            if dSq < minDistSq, minDistSq=dSq; bestPtSeg=ptSeg; bestPtBox=ptBox; end
        end
    end
    
    % 端点
    for ti = [0, L]
        ptSeg = p1+ti*dir;
        ptBox = max(aabbMin, min(ptSeg, aabbMax));
        dSq = sum((ptSeg-ptBox).^2);
        if dSq < minDistSq, minDistSq=dSq; bestPtSeg=ptSeg; bestPtBox=ptBox; end
    end
    
    % AABB边
    verts = [aabbMin(1) aabbMin(2) aabbMin(3);aabbMax(1) aabbMin(2) aabbMin(3);
             aabbMax(1) aabbMax(2) aabbMin(3);aabbMin(1) aabbMax(2) aabbMin(3);
             aabbMin(1) aabbMin(2) aabbMax(3);aabbMax(1) aabbMin(2) aabbMax(3);
             aabbMax(1) aabbMax(2) aabbMax(3);aabbMin(1) aabbMax(2) aabbMax(3)];
    edges = [1 2;2 3;3 4;4 1;5 6;6 7;7 8;8 5;1 5;2 6;3 7;4 8];
    for ei = 1:12
        e1=verts(edges(ei,1),:)'; e2=verts(edges(ei,2),:)';
        [dd,cp1,cp2] = segSegDist_v9(p1,p2,e1,e2);
        if dd^2 < minDistSq, minDistSq=dd^2; bestPtSeg=cp1; bestPtBox=cp2; end
    end
    
    dd = sqrt(minDistSq);
    d = dd - radius;
    if dd > 1e-8
        dirN = (bestPtBox - bestPtSeg)/dd;
        cpLink = bestPtSeg + dirN*radius;
    else
        cpLink = bestPtSeg;
    end
    cpBox = bestPtBox;
    
    % 穿透检测
    for t = [0.25, 0.5, 0.75]
        pt = p1 + t*L*dir;
        if all(pt >= aabbMin) && all(pt <= aabbMax)
            d = -radius - 0.001;
            return;
        end
    end
end

%% ---- v9.0 自碰撞检测 ----
function [ic, info, minDist, pairDists] = selfCollCheck_v9(outS, p)
    ic = false; info = ''; minDist = inf;
    pairDists = struct('name',{}, 'dist',{}, 'pt1',{}, 'pt2',{}, 'linkA',{}, 'linkB',{});
    
    pairs = {
        'Base',   'Elbow',    outS.base_bc1(:),outS.base_bc2(:),p.base.radius, outS.elbow_e1(:),outS.elbow_e2(:),p.elbow.radius;
        'Base',   'UpperArm', outS.base_bc1(:),outS.base_bc2(:),p.base.radius, outS.upperArm_ua1(:),outS.upperArm_ua2(:),p.upperArm.radius;
        'Base',   'Wrist',    outS.base_bc1(:),outS.base_bc2(:),p.base.radius, outS.wrist_wc(:),outS.wrist_wc(:),p.wrist.radius;
        'LowArm', 'UpperArm', outS.lowerArm_la1(:),outS.lowerArm_la2(:),p.lowerArm.radius, outS.upperArm_ua1(:),outS.upperArm_ua2(:),p.upperArm.radius;
        'LowArm', 'Wrist',    outS.lowerArm_la1(:),outS.lowerArm_la2(:),p.lowerArm.radius, outS.wrist_wc(:),outS.wrist_wc(:),p.wrist.radius;
        'Elbow',  'Wrist',    outS.elbow_e1(:),outS.elbow_e2(:),p.elbow.radius, outS.wrist_wc(:),outS.wrist_wc(:),p.wrist.radius;
        'Base',   'LowArm',   outS.base_bc1(:),outS.base_bc2(:),p.base.radius, outS.lowerArm_la1(:),outS.lowerArm_la2(:),p.lowerArm.radius;
        'Elbow',  'UpperArm', outS.elbow_e1(:),outS.elbow_e2(:),p.elbow.radius, outS.upperArm_ua1(:),outS.upperArm_ua2(:),p.upperArm.radius;
        'UprArm', 'Wrist',    outS.upperArm_ua1(:),outS.upperArm_ua2(:),p.upperArm.radius, outS.wrist_wc(:),outS.wrist_wc(:),p.wrist.radius;
    };
    
    for i = 1:size(pairs,1)
        nameA=pairs{i,1}; nameB=pairs{i,2};
        lp1=pairs{i,3}; lp2=pairs{i,4}; r1=pairs{i,5};
        lp3=pairs{i,6}; lp4=pairs{i,7}; r2=pairs{i,8};
        
        [d,cp1,cp2] = segSegDist_v9(lp1,lp2,lp3,lp4);
        netD = d - r1 - r2;
        
        entry.name = sprintf('%s-%s', nameA, nameB);
        entry.dist = netD;
        if d > 1e-6, dn = (cp2-cp1)/d; else, dn = [0;0;1]; end
        entry.pt1 = cp1 + dn*r1;
        entry.pt2 = cp2 - dn*r2;
        entry.linkA = nameA; entry.linkB = nameB;
        pairDists(end+1) = entry;
        
        if netD < minDist, minDist = netD; end
        if netD < 0 && ~ic
            ic = true; info = sprintf('%s-%s(%.3fm)', nameA, nameB, netD);
        end
    end
end

%% ---- v9.0 环境碰撞检测 (解析法) ----
function [ic, info, minDist, detailDists] = envCollCheck_v9(outS, p, envObjs)
    ic = false; info = ''; minDist = inf;
    detailDists = struct('name',{}, 'dist',{}, 'pt1',{}, 'pt2',{}, 'link',{}, 'obj',{});
    
    links = {
        'Base',     outS.base_bc1(:),      outS.base_bc2(:),      p.base.radius;
        'LowerArm', outS.lowerArm_la1(:),  outS.lowerArm_la2(:),  p.lowerArm.radius;
        'Elbow',    outS.elbow_e1(:),       outS.elbow_e2(:),       p.elbow.radius;
        'UpperArm', outS.upperArm_ua1(:),  outS.upperArm_ua2(:),  p.upperArm.radius;
        'Wrist',    outS.wrist_wc(:),       outS.wrist_wc(:),       p.wrist.radius;
    };
    
    for li = 1:size(links,1)
        linkName = links{li,1};
        lp1 = links{li,2}; lp2 = links{li,3}; lr = links{li,4};
        
        for oi = 1:length(envObjs)
            obj = envObjs(oi);
            if strcmp(linkName,'Base') && (strcmp(obj.type,'ground') || strcmp(obj.name,'Cabinet'))
                continue;
            end
            
            [d, cpL, cpB] = segAABBDistFull_v9(lp1, lp2, lr, obj.aabbMin(:), obj.aabbMax(:));
            
            entry.name = sprintf('%s>%s', linkName, obj.name);
            entry.dist = d; entry.pt1 = cpL; entry.pt2 = cpB;
            entry.link = linkName; entry.obj = obj.name;
            detailDists(end+1) = entry;
            
            if d < minDist, minDist = d; end
            if d < 0 && ~ic
                ic = true; info = sprintf('%s>%s(%.3fm)', linkName, obj.name, d);
            end
        end
    end
end

%% ---- 碰撞热力图渲染 (低面数) ----
function drawCollisionHeatmap_v9(outS, params, selfPairDists, envDetailDists, ...
    threshD, threshW, threshC, sphereN, cylN)
    
    linkNames = {'Base','LowerArm','Elbow','UpperArm','Wrist'};
    linkMinDist = inf(1,5);
    
    for i=1:length(selfPairDists)
        sp = selfPairDists(i);
        for li=1:5
            if contains(sp.name, linkNames{li}) && sp.dist < linkMinDist(li)
                linkMinDist(li) = sp.dist;
            end
        end
    end
    for i=1:length(envDetailDists)
        ed = envDetailDists(i);
        for li=1:5
            if strcmp(ed.link, linkNames{li}) && ed.dist < linkMinDist(li)
                linkMinDist(li) = ed.dist;
            end
        end
    end
    
    linkSegs = {
        outS.base_bc1(:),outS.base_bc2(:),params.base.radius;
        outS.lowerArm_la1(:),outS.lowerArm_la2(:),params.lowerArm.radius;
        outS.elbow_e1(:),outS.elbow_e2(:),params.elbow.radius;
        outS.upperArm_ua1(:),outS.upperArm_ua2(:),params.upperArm.radius;
        outS.wrist_wc(:),outS.wrist_wc(:),params.wrist.radius;
    };
    
    for li=1:5
        d = linkMinDist(li);
        if d<=threshD,     col=[1 0 0]; alp=0.45;
        elseif d<=threshW, t=d/threshW; col=[1 t*0.8 0]; alp=0.30;
        elseif d<=threshC, t=(d-threshW)/(threshC-threshW); col=[1-t*0.5 0.8+t*0.2 t*0.2]; alp=0.22;
        else,              col=[0.2 0.85 0.3]; alp=0.12;
        end
        drawCapsule_v9(linkSegs{li,1}, linkSegs{li,2}, linkSegs{li,3}*1.05, col, alp, sphereN, cylN);
    end
end

%% ---- 绘制胶囊体 (低面数) ----
function drawCapsule_v9(p1, p2, r, color, alpha_val, sphereN, cylN)
    if norm(p2-p1) < 1e-6
        [X,Y,Z] = sphere(sphereN);
        surf(X*r+p1(1), Y*r+p1(2), Z*r+p1(3), ...
            'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha_val);
        return;
    end
    
    v = p2 - p1; L = norm(v);
    [X,Y,Z] = cylinder(r, cylN); Z = Z*L;
    
    dd=[0;0;1]; td=v/L; cp=cross(dd,td);
    if norm(cp)>1e-6
        R=axang2r_v9([cp'/norm(cp), acos(max(-1,min(1,dot(dd,td))))]);
    else
        R=eye(3); if dot(dd,td)<0, R(3,3)=-1; R(1,1)=-1; end
    end
    
    for i=1:numel(X)
        pt=R*[X(i);Y(i);Z(i)]; X(i)=pt(1)+p1(1); Y(i)=pt(2)+p1(2); Z(i)=pt(3)+p1(3);
    end
    surf(X,Y,Z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha_val);
    
    [SX,SY,SZ] = sphere(sphereN);
    surf(SX*r+p1(1),SY*r+p1(2),SZ*r+p1(3), 'FaceColor',color,'EdgeColor','none','FaceAlpha',alpha_val);
    surf(SX*r+p2(1),SY*r+p2(2),SZ*r+p2(3), 'FaceColor',color,'EdgeColor','none','FaceAlpha',alpha_val);
end

%% ---- 最小距离连线 ----
function drawMinDistLine(pairDists, threshC, lineColor, fontName)
    if isempty(pairDists), return; end
    [~, sIdx] = min([pairDists.dist]);
    sp = pairDists(sIdx);
    if sp.dist < threshC
        plot3([sp.pt1(1) sp.pt2(1)], [sp.pt1(2) sp.pt2(2)], [sp.pt1(3) sp.pt2(3)], ...
            '-', 'Color', lineColor, 'LineWidth', 2.5);
        mid = (sp.pt1 + sp.pt2)/2;
        text(mid(1), mid(2), mid(3)+0.05, sprintf('%.0fmm', sp.dist*1000), ...
            'FontSize', 10, 'FontWeight', 'bold', 'Color', lineColor, ...
            'BackgroundColor', [1 1 0.85 0.9], 'HorizontalAlignment', 'center', ...
            'FontName', fontName);
    end
end

%% ---- v9.0 仪表盘 (英文标识+中文兼容) ----
function drawDashboard_v9(ax, idx, N, label, pc, threshW, threshC, cjkFont, monoFont)
    axes(ax);
    
    % 背景
    rectangle('Position', [0 0 1 1], 'FaceColor', [0.98 0.98 0.98], ...
        'EdgeColor', [0.65 0.65 0.65], 'LineWidth', 1.5, 'Curvature', 0.02);
    
    yP = 0.96;
    
    % 标题
    text(0.5, yP, 'Collision Dashboard', 'FontSize', 13, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'Color', [0.15 0.15 0.25], 'FontName', cjkFont);
    yP = yP - 0.035;
    text(0.5, yP, sprintf('[%d/%d] %s', idx, N, label), 'FontSize', 10, ...
        'HorizontalAlignment', 'center', 'Color', [0.4 0.4 0.4], 'FontName', cjkFont);
    yP = yP - 0.05;
    
    % 综合状态
    if pc.isColl
        sCol=[0.9 0.1 0.1]; sTxt='COLLISION'; sBg=[1 0.92 0.92];
    elseif pc.isWarn
        sCol=[0.85 0.55 0]; sTxt='WARNING'; sBg=[1 1 0.90];
    elseif pc.isCaut
        sCol=[0.7 0.5 0.1]; sTxt='CAUTION'; sBg=[1 0.97 0.90];
    else
        sCol=[0 0.6 0.2]; sTxt='SAFE'; sBg=[0.92 1 0.92];
    end
    
    rectangle('Position', [0.05 yP-0.035 0.9 0.055], 'FaceColor', sBg, ...
        'EdgeColor', sCol, 'LineWidth', 2, 'Curvature', 0.3);
    text(0.5, yP-0.008, sprintf('%s  Min: %.1fmm', sTxt, pc.overallMin*1000), ...
        'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
        'Color', sCol, 'FontName', cjkFont);
    yP = yP - 0.07;
    
    % --- Layer 1: Self-Collision ---
    text(0.05, yP, 'Layer 1: Self-Collision', 'FontSize', 10, 'FontWeight', 'bold', ...
        'Color', [0.2 0.2 0.6], 'FontName', cjkFont);
    yP = yP - 0.028;
    
    if pc.sc
        text(0.08, yP, sprintf('COLLISION: %s', pc.selfInfo), 'FontSize', 9, ...
            'Color', [0.8 0 0], 'FontName', cjkFont);
    else
        text(0.08, yP, sprintf('OK  Min: %.1fmm', pc.selfMinDist*1000), ...
            'FontSize', 9, 'Color', [0 0.5 0], 'FontName', cjkFont);
    end
    yP = yP - 0.025;
    
    if ~isempty(pc.selfPairDists)
        dists = [pc.selfPairDists.dist];
        [~, sI] = sort(dists);
        nShow = min(5, length(sI));
        for i=1:nShow
            sp = pc.selfPairDists(sI(i));
            if sp.dist<0, c=[0.8 0 0]; mk='X';
            elseif sp.dist<threshW, c=[0.8 0.5 0]; mk='!';
            else, c=[0.3 0.55 0.3]; mk='o';
            end
            text(0.08, yP, sprintf('[%s] %-12s %6.1fmm', mk, sp.name, sp.dist*1000), ...
                'FontSize', 8, 'FontName', monoFont, 'Color', c);
            yP = yP - 0.02;
        end
    end
    yP = yP - 0.008;
    
    % --- Layer 2: Env-Collision ---
    text(0.05, yP, 'Layer 2: Env-Collision', 'FontSize', 10, 'FontWeight', 'bold', ...
        'Color', [0.5 0.15 0.15], 'FontName', cjkFont);
    yP = yP - 0.028;
    
    if pc.ec
        text(0.08, yP, sprintf('COLLISION: %s', pc.envInfo), 'FontSize', 9, ...
            'Color', [0.8 0 0], 'FontName', cjkFont);
    else
        text(0.08, yP, sprintf('OK  Min: %.1fmm', pc.envMinDist*1000), ...
            'FontSize', 9, 'Color', [0 0.5 0], 'FontName', cjkFont);
    end
    yP = yP - 0.025;
    
    if ~isempty(pc.envDetailDists)
        dists = [pc.envDetailDists.dist];
        [~, sI] = sort(dists);
        nShow = min(6, length(sI));
        for i=1:nShow
            ed = pc.envDetailDists(sI(i));
            if ed.dist<0, c=[0.8 0 0]; mk='X';
            elseif ed.dist<threshW, c=[0.8 0.5 0]; mk='!';
            elseif ed.dist<threshC, c=[0.6 0.45 0.1]; mk='~';
            else, c=[0.3 0.55 0.3]; mk='o';
            end
            text(0.08, yP, sprintf('[%s] %-18s %6.1fmm', mk, ed.name, ed.dist*1000), ...
                'FontSize', 8, 'FontName', monoFont, 'Color', c);
            yP = yP - 0.02;
        end
    end
    yP = yP - 0.008;
    
    % --- Layer 3: TCP ---
    text(0.05, yP, 'Layer 3: TCP Status', 'FontSize', 10, 'FontWeight', 'bold', ...
        'Color', [0.15 0.45 0.15], 'FontName', cjkFont);
    yP = yP - 0.028;
    
    text(0.08, yP, sprintf('TCP: [%.3f, %.3f, %.3f]m', pc.ep), ...
        'FontSize', 9, 'Color', [0.3 0.3 0.3], 'FontName', cjkFont);
    yP = yP - 0.022;
    
    if pc.tcpOnGround
        text(0.08, yP, 'TCP ON GROUND!', 'FontSize', 9, 'Color', [0.8 0 0], 'FontName', cjkFont);
    else
        text(0.08, yP, sprintf('Height: %.0fmm  OK', pc.ep(3)*1000), ...
            'FontSize', 9, 'Color', [0 0.5 0], 'FontName', cjkFont);
    end
    yP = yP - 0.03;
    
    % --- 距离条形图 ---
    text(0.05, yP, 'Link Safety Distance', 'FontSize', 10, 'FontWeight', 'bold', ...
        'Color', [0.25 0.25 0.45], 'FontName', cjkFont);
    yP = yP - 0.025;
    
    linkNames = {'Base','LowArm','Elbow','UprArm','Wrist'};
    linkMinD = inf(1,5);
    fullNames = {'Base','LowerArm','Elbow','UpperArm','Wrist'};
    
    for i=1:length(pc.selfPairDists)
        sp = pc.selfPairDists(i);
        for li=1:5
            if contains(sp.name, linkNames{li}) && sp.dist<linkMinD(li)
                linkMinD(li) = sp.dist;
            end
        end
    end
    for i=1:length(pc.envDetailDists)
        ed = pc.envDetailDists(i);
        for li=1:5
            if strcmp(ed.link, fullNames{li}) && ed.dist<linkMinD(li)
                linkMinD(li) = ed.dist;
            end
        end
    end
    
    maxBar = 0.3;
    barH = 0.02;
    for li=1:5
        d = min(linkMinD(li), maxBar);
        barW = max(0.01, (d/maxBar)*0.65);
        
        if linkMinD(li)<=0, bCol=[0.9 0.15 0.15];
        elseif linkMinD(li)<threshW, bCol=[0.9 0.6 0.1];
        elseif linkMinD(li)<threshC, bCol=[0.8 0.8 0.2];
        else, bCol=[0.2 0.75 0.3];
        end
        
        rectangle('Position',[0.24 yP-barH/2 0.65 barH], 'FaceColor',[0.93 0.93 0.93], 'EdgeColor','none');
        rectangle('Position',[0.24 yP-barH/2 barW barH], 'FaceColor',bCol, 'EdgeColor','none','Curvature',0.3);
        
        text(0.04, yP, linkNames{li}, 'FontSize', 9, 'Color', [0.25 0.25 0.25], 'FontName', cjkFont);
        
        if linkMinD(li) < inf
            text(0.91, yP, sprintf('%.0f', linkMinD(li)*1000), 'FontSize', 8, ...
                'HorizontalAlignment','right', 'Color', bCol, 'FontWeight','bold', 'FontName', cjkFont);
        end
        yP = yP - 0.032;
    end
    
    xlim([0 1]); ylim([0 1]);
end

%% ---- TCP框架内检测 ----
function inF = isTCPInFrame(tcp, fr)
    inF = tcp(1)>fr.cx-fr.widthX/2 && tcp(1)<fr.cx+fr.widthX/2 && ...
          tcp(2)>fr.cy-fr.depthY/2 && tcp(2)<fr.cy+fr.depthY/2 && ...
          tcp(3)<fr.height;
end

%% ---- 预计算碰撞点 (无绘图) ----
function outS = computeCollisionPoints(Tf, params)
    outS = struct();
    T00=Tf{1}; T02=Tf{3}; T03=Tf{4}; T04=Tf{5}; T05=Tf{6};
    
    bs=params.base.start(:); be=params.base.('end'); be=be(:);
    outS.base_bc1=T00(1:3,1:3)*bs+T00(1:3,4); outS.base_bc2=T00(1:3,1:3)*be+T00(1:3,4);
    
    ls=params.lowerArm.start(:); le=params.lowerArm.('end'); le=le(:);
    outS.lowerArm_la1=T02(1:3,1:3)*ls+T02(1:3,4); outS.lowerArm_la2=T02(1:3,1:3)*le+T02(1:3,4);
    
    es=params.elbow.start(:); ee=params.elbow.('end'); ee=ee(:);
    outS.elbow_e1=T03(1:3,1:3)*es+T03(1:3,4); outS.elbow_e2=T03(1:3,1:3)*ee+T03(1:3,4);
    
    us=params.upperArm.start(:); ue=params.upperArm.('end'); ue=ue(:);
    outS.upperArm_ua1=T04(1:3,1:3)*us+T04(1:3,4); outS.upperArm_ua2=T04(1:3,1:3)*ue+T04(1:3,4);
    
    outS.wrist_wc=T05(1:3,1:3)*params.wrist.offset(:)+T05(1:3,4);
end

%% ---- 几何工具 ----
function [d, cp1, cp2] = segSegDist_v9(p1, p2, p3, p4)
    d1=p2-p1; d2=p4-p3; r=p1-p3;
    a=dot(d1,d1); b=dot(d1,d2); c=dot(d2,d2); dd=dot(d1,r); e=dot(d2,r);
    dn=a*c-b*b;
    if dn<1e-10, s=0; t=dd/max(b,1e-10);
    else, s=(b*e-c*dd)/dn; t=(a*e-b*dd)/dn; end
    s=max(0,min(1,s)); t=max(0,min(1,t));
    cp1=p1+s*d1; cp2=p3+t*d2; d=norm(cp1-cp2);
end

function linkSegs = getLinkSegments(outS, params)
    linkSegs = struct('p1',{}, 'p2',{}, 'r',{});
    linkSegs(1).p1=outS.base_bc1(:); linkSegs(1).p2=outS.base_bc2(:); linkSegs(1).r=params.base.radius;
    linkSegs(2).p1=outS.lowerArm_la1(:); linkSegs(2).p2=outS.lowerArm_la2(:); linkSegs(2).r=params.lowerArm.radius;
    linkSegs(3).p1=outS.elbow_e1(:); linkSegs(3).p2=outS.elbow_e2(:); linkSegs(3).r=params.elbow.radius;
    linkSegs(4).p1=outS.upperArm_ua1(:); linkSegs(4).p2=outS.upperArm_ua2(:); linkSegs(4).r=params.upperArm.radius;
    linkSegs(5).p1=outS.wrist_wc(:); linkSegs(5).p2=outS.wrist_wc(:); linkSegs(5).r=params.wrist.radius;
end

function cmap = customHeatmap()
    n=256; cmap=zeros(n,3);
    for i=1:n
        t=(i-1)/(n-1);
        if t<0.15, cmap(i,:)=[0.8 0 0];
        elseif t<0.3, s=(t-0.15)/0.15; cmap(i,:)=[1 s*0.6 0];
        elseif t<0.5, s=(t-0.3)/0.2; cmap(i,:)=[1-s*0.2 0.6+s*0.3 s*0.1];
        elseif t<0.7, s=(t-0.5)/0.2; cmap(i,:)=[0.8-s*0.6 0.9 0.1+s*0.4];
        else, s=(t-0.7)/0.3; cmap(i,:)=[0.2-s*0.1 0.9-s*0.2 0.5+s*0.3];
        end
    end
end

%% ---- IK搜索 ----
function [q2b,q3b,q4b,eb] = searchIK_ext(q1, tr, tz)
    global d1 d2 d3 d4 d5 d6 a2 a3 T6T;
    eb=inf; q2b=-pi/2; q3b=pi/2; q4b=-pi/2;
    for q2d=-175:3:-30
        for q3d=10:3:165
            for q4d=[-150,-135,-120,-90,-60,-45]
                q2=q2d*pi/180; q3=q3d*pi/180; q4=q4d*pi/180;
                [~,~,~,~,~,~,T]=FK_SSerial([q1,q2,q3,q4,-pi/2,0]);
                r_fk=sqrt(T(1,4)^2+T(2,4)^2); z_fk=T(3,4);
                pe=sqrt((r_fk-tr)^2+(z_fk-tz)^2);
                ez=T(1:3,3); zd=acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                tot=pe+0.08*zd;
                if tot<eb && zd<30*pi/180, eb=pe; q2b=q2; q3b=q3; q4b=q4; end
            end
        end
    end
    q2c=rad2deg(q2b); q3c=rad2deg(q3b); q4c=rad2deg(q4b);
    for q2d=(q2c-5):0.5:(q2c+5)
        for q3d=(q3c-5):0.5:(q3c+5)
            for q4d=(q4c-10):2:(q4c+10)
                if q2d<-175||q2d>-30||q3d<10||q3d>165, continue; end
                q2=q2d*pi/180; q3=q3d*pi/180; q4=q4d*pi/180;
                [~,~,~,~,~,~,T]=FK_SSerial([q1,q2,q3,q4,-pi/2,0]);
                r_fk=sqrt(T(1,4)^2+T(2,4)^2); z_fk=T(3,4);
                pe=sqrt((r_fk-tr)^2+(z_fk-tz)^2);
                ez=T(1:3,3); zd=acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                tot=pe+0.08*zd;
                if tot<eb && zd<30*pi/180, eb=pe; q2b=q2; q3b=q3; q4b=q4; end
            end
        end
    end
end

%% ---- 基础绘图 (v9.0 低面数) ----
function s = iff(c,a,b), if c, s=a; else, s=b; end, end

function drawGround_v9(x0,x1,y0,y1)
    patch('Vertices',[x0 y0 0;x1 y0 0;x1 y1 0;x0 y1 0],'Faces',[1 2 3 4],...
          'FaceColor',[.92 .92 .90],'EdgeColor','none','FaceAlpha',0.4);
end

function drawCabinet_v9(c, bx, by, fontName)
    x=bx-c.widthX/2; y=by-c.depthY/2; w=c.widthX; d=c.depthY; h=c.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',c.color,'EdgeColor',[.5 .5 .5],'FaceAlpha',.95,'LineWidth',1);
    sv=[x+.06 y+d-.01 h*.55; x+w-.06 y+d-.01 h*.55; x+w-.06 y+d-.01 h*.80; x+.06 y+d-.01 h*.80];
    patch('Vertices',sv,'Faces',[1 2 3 4],'FaceColor',[.6 .9 .6],'EdgeColor','k','LineWidth',1.5);
end

function drawFrame_v9(f, cylN)
    r=f.tubeR; wx=f.widthX; dy=f.depthY; h=f.height;
    cx=f.cx; cy=f.cy;
    c=[cx-wx/2 cy-dy/2; cx+wx/2 cy-dy/2; cx+wx/2 cy+dy/2; cx-wx/2 cy+dy/2];
    for i=1:4, drawTube_v9(c(i,1),c(i,2),0,c(i,1),c(i,2),h,r,f.color,cylN); end
    edges={[1,2],[2,3],[3,4],[4,1]};
    for hz=[0.05 h/3 2*h/3 h-0.05]
        for ei=2:4  % skip edge 1 (open -Y)
            i1=edges{ei}(1); i2=edges{ei}(2);
            drawTube_v9(c(i1,1),c(i1,2),hz,c(i2,1),c(i2,2),hz,r*.8,f.color,cylN);
        end
    end
    % Diagonal bracing (simplified - fewer segments)
    for ei=2:4
        i1=edges{ei}(1); i2=edges{ei}(2);
        x1=c(i1,1); y1=c(i1,2); x2=c(i2,1); y2=c(i2,2);
        for k=0:3  % v8 had 0:7, reduced
            z1=k*h/4+0.05; z2=z1+h/8;
            mx=(x1+x2)/2; my=(y1+y2)/2;
            drawTube_v9(x1,y1,z1,mx,my,z2,r*.18,f.color*.85,cylN);
            drawTube_v9(mx,my,z2,x2,y2,z1+h/4-0.05,r*.18,f.color*.85,cylN);
        end
    end
end

function drawPallet_v9(pal, frm, fontName)
    x=frm.cx-pal.widthX/2; y=frm.cy-pal.depthY/2;
    w=pal.widthX; d=pal.depthY; h=pal.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',pal.color,'EdgeColor',[.15 .30 .60],'FaceAlpha',.90,'LineWidth',1.2);
    text(frm.cx, frm.cy, h+0.03, sprintf('Pallet %dcm', round(h*100)), 'FontSize',8, ...
         'HorizontalAlignment','center', 'Color',[.05 .15 .45], 'FontWeight','bold', 'FontName', fontName);
end

function drawConveyor_v9(cv, cylN)
    cx=cv.cx; cy=cv.cy; ly=cv.lengthY; wx=cv.widthX; hz=cv.heightZ;
    x0=cx-wx/2; y0=cy-ly/2;
    drawBox3D_v9(x0-.015,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    drawBox3D_v9(x0+wx,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    lw=.035; yL=[y0+.2 cy y0+ly-.2];
    for yi=1:3
        for s=[-1 1]
            lx=cx+s*(wx/2-.06);
            drawBox3D_v9(lx-lw/2,yL(yi)-lw/2,0,lw,lw,hz-.01,[.25 .25 .25]);
        end
        drawBox3D_v9(x0+.04,yL(yi)-lw/2,hz*.35,wx-.08,lw,lw,[.25 .25 .25]);
    end
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        drawRoller_v9(cx,ry,hz,wx*.9,cv.rollerR,[.55 .55 .55],cylN);
    end
    bz=hz+cv.rollerR;
    bV=[x0+.02 y0+.04 bz;x0+wx-.02 y0+.04 bz;x0+wx-.02 y0+ly-.04 bz;x0+.02 y0+ly-.04 bz;
        x0+.02 y0+.04 bz+cv.beltH;x0+wx-.02 y0+.04 bz+cv.beltH;
        x0+wx-.02 y0+ly-.04 bz+cv.beltH;x0+.02 y0+ly-.04 bz+cv.beltH];
    patch('Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[.15 .15 .15],'FaceAlpha',.92);
end

function drawRoller_v9(cx,y,z,w,r,col,cylN)
    [X,Y,Z]=cylinder(r,cylN); Z=Z*w-w/2;
    surf(Z+cx,zeros(size(X))+y,X+z+r,'FaceColor',col,'EdgeColor','none','FaceAlpha',.6);
end

function drawBox3D_v9(x,y,z,dx,dy,dz,col)
    v=[x y z;x+dx y z;x+dx y+dy z;x y+dy z;x y z+dz;x+dx y z+dz;x+dx y+dy z+dz;x y+dy z+dz];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',col,'EdgeColor','none','FaceAlpha',.95);
end

function drawBox_v9(pos, bx)
    x=pos(1)-bx.lx/2; y=pos(2)-bx.wy/2; z=pos(3)-bx.hz/2;
    v=[x y z;x+bx.lx y z;x+bx.lx y+bx.wy z;x y+bx.wy z;
       x y z+bx.hz;x+bx.lx y z+bx.hz;x+bx.lx y+bx.wy z+bx.hz;x y+bx.wy z+bx.hz];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',bx.color,'EdgeColor',[.35 .25 .1],'FaceAlpha',.95,'LineWidth',1.2);
end

function drawTube_v9(x1,y1,z1,x2,y2,z2,radius,color,cylN)
    [X,Y,Z]=cylinder(radius,cylN);
    v=[x2-x1;y2-y1;z2-z1]; l=norm(v);
    if l<.001, return; end
    Z=Z*l;
    dd=[0;0;1]; td=v/l; cp=cross(dd,td);
    if norm(cp)>1e-6
        R=axang2r_v9([cp'/norm(cp), acos(max(-1,min(1,dot(dd,td))))]);
    else
        R=eye(3); if dot(dd,td)<0, R(3,3)=-1; R(1,1)=-1; end
    end
    for i=1:numel(X)
        pt=R*[X(i);Y(i);Z(i)]; X(i)=pt(1)+x1; Y(i)=pt(2)+y1; Z(i)=pt(3)+z1;
    end
    surf(X,Y,Z,'FaceColor',color,'EdgeColor','none','FaceAlpha',.85);
end

function R=axang2r_v9(ax)
    a=ax(1:3); g=ax(4);
    c=cos(g); s=sin(g); t=1-c;
    x=a(1); y=a(2); z=a(3);
    R=[t*x*x+c t*x*y-s*z t*x*z+s*y;t*x*y+s*z t*y*y+c t*y*z-s*x;t*x*z-s*y t*y*z+s*x t*z*z+c];
end
