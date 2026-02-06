%% testS50_Palletizing_v8.m - HR_S50-2000 码垛工作站 v8.0 (完整碰撞检测增强版)
%  3箱连续码垛演示: 传送带(-Y→+Y) → 蓝框(+Y方向)
%  ★ v8.0 新增: 三层碰撞检测 + 距离色阶可视化 + 碰撞仪表盘
%
%  碰撞检测三层体系:
%    1. 连杆-连杆 自碰撞 (5连杆体: 基座/下臂/肘/上臂/腕)
%    2. 连杆-环境 碰撞 (所有连杆 vs 地面/电箱/框架/传送带/托盘/OBB面)
%    3. TCP-区域 碰撞 (TCP是否侵入禁区)
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

close all; clear all; clc;
addpath('collisionVisual'); addpath(genpath('collisionVisual'));

%% ====================== 环境设置 ======================
isHeadless = ~usejava('desktop');
outputDir = './pic/S50_palletizing_v8';
if isHeadless, set(0, 'DefaultFigureVisible', 'off'); end
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

%% ====================== 场景几何参数 (m) ======================
% 电箱
cab.widthX  = 0.55;
cab.depthY  = 0.65;
cab.heightZ = 0.80;
cab.color   = [0.95, 0.95, 0.93];

% 蓝色笼式框架 (+Y方向, 开口朝-Y)
frame.widthX  = 1.20;
frame.depthY  = 1.00;
frame.height  = 2.00;
frame.tubeR   = 0.030;
frame.color   = [0.25, 0.55, 0.85];

% 蓝色托盘 (高度0.55m)
pallet.widthX  = 1.00;
pallet.depthY  = 0.80;
pallet.heightZ = 0.55;
pallet.color   = [0.20, 0.45, 0.80];

% 传送带 (+X方向, 沿Y延伸)
conv.lengthY  = 2.00;
conv.widthX   = 0.55;
conv.heightZ  = 0.75;
conv.beltH    = 0.035;
conv.rollerR  = 0.030;
conv.nRollers = 12;
conv.color    = [0.30, 0.30, 0.32];

% 箱子
box.lx = 0.40; box.wy = 0.30; box.hz = 0.25;
box.color = [0.65, 0.45, 0.25];

nBoxes = 3;

%% ====================== 碰撞安全参数 (v8.0) ======================
COLL_THRESH_DANGER  = 0.00;   % 碰撞阈值 [m]
COLL_THRESH_WARNING = 0.05;   % 警告阈值 [m]
COLL_THRESH_CAUTION = 0.10;   % 注意阈值 [m]
GROUND_Z = 0.0;                % 地面高度 [m]

%% ====================== 布局 ======================
baseX = 0.0; baseY = 0.0;
baseZ = cab.heightZ;

% 蓝框
frameGap = 0.40;
frame.cx = 0.0;
frame.cy = cab.depthY/2 + frameGap + frame.depthY/2;

% 传送带
convGap = 0.40;
conv.cx = cab.widthX/2 + convGap + conv.widthX/2;
conv.cy = -0.30;

convSurfZ = conv.heightZ + conv.rollerR + conv.beltH;
convBoxTopZ = convSurfZ + box.hz;
palletSurfZ = pallet.heightZ;

fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  HR_S50-2000 码垛 v8.0 — 三层碰撞检测增强版 (3箱连续码垛)         ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n\n');

fprintf('📐 场景布局:\n');
fprintf('   电箱: (%.2f,%.2f) %.2f×%.2f×%.2fm\n', baseX,baseY,cab.widthX,cab.depthY,cab.heightZ);
fprintf('   蓝框: (%.2f,%.2f) 开口-Y  托盘高:%.2fm(蓝色)\n', frame.cx,frame.cy,pallet.heightZ);
fprintf('   传送带: (%.2f,%.2f) 范围Y=[%.2f,%.2f] 皮带面z=%.3fm\n', ...
    conv.cx, conv.cy, conv.cy-conv.lengthY/2, conv.cy+conv.lengthY/2, convSurfZ);
fprintf('   箱顶z=%.3fm  托盘面z=%.3fm  基座高:%.2fm\n\n', convBoxTopZ, palletSurfZ, baseZ);

% 间距验证
cabFront = baseY + cab.depthY/2;
frameBack = frame.cy - frame.depthY/2;
gap1 = frameBack - cabFront;
cabRight = baseX + cab.widthX/2;
convLeft = conv.cx - conv.widthX/2;
gap2 = convLeft - cabRight;
fprintf('   间距: 电箱↔蓝框=%.2fm  电箱↔传送带=%.2fm ✅\n\n', gap1, gap2);

%% ====================== 传送带上3个箱子的位置 ======================
convBoxY = zeros(1, nBoxes);
yStart = conv.cy - 0.15;
ySpacing = 0.45;
for bi = 1:nBoxes
    convBoxY(bi) = yStart - (bi-1)*ySpacing;
end
convBoxX = conv.cx;

fprintf('📦 传送带箱子位置:\n');
for bi=1:nBoxes
    fprintf('   箱%d: [%.3f, %.3f, %.3f]\n', bi, convBoxX, convBoxY(bi), convBoxTopZ);
end
fprintf('\n');

%% ====================== 蓝框内码垛位置 ======================
placePos = zeros(nBoxes, 3);
layer1_z = palletSurfZ + box.hz;
placePos(1,:) = [frame.cx - box.lx/2, frame.cy, layer1_z];
placePos(2,:) = [frame.cx + box.lx/2, frame.cy, layer1_z];
layer2_z = palletSurfZ + 2*box.hz;
placePos(3,:) = [frame.cx, frame.cy, layer2_z];

fprintf('📦 蓝框内码垛位置:\n');
for bi=1:nBoxes
    fprintf('   箱%d: [%.3f, %.3f, %.3f] 第%d层\n', bi, placePos(bi,:), 1+(bi>2));
end
fprintf('\n');

%% ====================== 加载碰撞模型 ======================
params = readCollisionModelJson("./model/collideConfig/S50_collision.json");
params_tool = readToolCollisionJson("./model/collideConfig/nonetool_collision.json");
fprintf('✅ 碰撞模型加载\n\n');

global d1 d2 d3 d4 d5 d6 a2 a3 T6T;
d1=params.DH.d1; d2=params.DH.d2; d3=params.DH.d3;
d4=params.DH.d4; d5=params.DH.d5; d6=params.DH.d6;
a2=-params.DH.a2; a3=-params.DH.a3; T6T=eye(4);

Tb = eye(4); Tb(1,4)=baseX; Tb(2,4)=baseY; Tb(3,4)=baseZ;

%% ====================== 构建环境碰撞模型 (v8.0) ======================
% 将所有场景物体转化为 AABB/OBB 列表用于碰撞检测
envObjs = buildEnvironmentModel(cab, frame, pallet, conv, baseX, baseY, GROUND_Z);
fprintf('🏗️  环境碰撞模型: %d 个碰撞体\n', length(envObjs));
for ei = 1:length(envObjs)
    fprintf('   [%d] %s  包围盒: [%.2f,%.2f,%.2f]~[%.2f,%.2f,%.2f]\n', ...
        ei, envObjs(ei).name, envObjs(ei).aabbMin, envObjs(ei).aabbMax);
end
fprintf('\n');

%% ====================== IK求解 ======================
fprintf('🔍 IK求解 (每箱独立)...\n');

pickIK = struct('q1',{},'q2',{},'q3',{},'q4',{},'err',{});
for bi = 1:nBoxes
    dx = convBoxX - baseX;
    dy = convBoxY(bi) - baseY;
    q1 = atan2(-dy, -dx);
    tr = sqrt(dx^2 + dy^2);
    tz = convBoxTopZ - baseZ;
    [q2,q3,q4,err] = searchIK_ext(q1, tr, tz);
    pickIK(bi).q1=q1; pickIK(bi).q2=q2; pickIK(bi).q3=q3; pickIK(bi).q4=q4; pickIK(bi).err=err;
    fprintf('   取箱%d: J1=%+6.1f° err=%.1fmm\n', bi, rad2deg(q1), err*1000);
end

placeIK = struct('q1',{},'q2',{},'q3',{},'q4',{},'err',{});
for bi = 1:nBoxes
    dx = placePos(bi,1) - baseX;
    dy = placePos(bi,2) - baseY;
    q1 = atan2(-dy, -dx);
    tr = sqrt(dx^2 + dy^2);
    tz = placePos(bi,3) - baseZ;
    [q2,q3,q4,err] = searchIK_ext(q1, tr, tz);
    placeIK(bi).q1=q1; placeIK(bi).q2=q2; placeIK(bi).q3=q3; placeIK(bi).q4=q4; placeIK(bi).err=err;
    fprintf('   放箱%d: J1=%+6.1f° err=%.1fmm\n', bi, rad2deg(q1), err*1000);
end
fprintf('\n');

%% ====================== 生成全部姿态序列 ======================
allPoses = {};
frameIdx = 0;
q_home = [pickIK(1).q1, -pi/3, pi/4, 0, -pi/3, 0];

frameIdx = frameIdx+1;
allPoses{frameIdx} = struct('name','初始待机', 'q', q_home, 'chk','none', ...
    'boxId',0, 'placed',0, 'convRemain',1:nBoxes, 'carrying',false);

for bi = 1:nBoxes
    pk = pickIK(bi);
    pl = placeIK(bi);
    q_grasp   = [pk.q1, pk.q2, pk.q3, pk.q4, -pi/2, 0];
    q_release = [pl.q1, pl.q2, pl.q3, pl.q4, -pi/2, 0];
    
    q_hov_pk  = q_grasp;
    q_hov_pk(2) = q_hov_pk(2) + deg2rad(8);
    q_hov_pk(3) = q_hov_pk(3) - deg2rad(6);
    
    q_hov_pl  = q_release;
    q_hov_pl(2) = q_hov_pl(2) + deg2rad(8);
    q_hov_pl(3) = q_hov_pl(3) - deg2rad(6);
    
    convRemainBefore = bi:nBoxes;
    convRemainAfter  = (bi+1):nBoxes;
    placedBefore = bi - 1;
    placedAfter  = bi;
    
    frameIdx=frameIdx+1;
    allPoses{frameIdx} = struct('name',sprintf('悬停取箱%d',bi), ...
        'q',q_hov_pk, 'chk','none', 'boxId',bi, ...
        'placed',placedBefore, 'convRemain',convRemainBefore, 'carrying',false);
    
    frameIdx=frameIdx+1;
    allPoses{frameIdx} = struct('name',sprintf('取箱%d',bi), ...
        'q',q_grasp, 'chk','pick', 'boxId',bi, ...
        'placed',placedBefore, 'convRemain',convRemainBefore, 'carrying',false);
    
    frameIdx=frameIdx+1;
    allPoses{frameIdx} = struct('name',sprintf('提升箱%d',bi), ...
        'q',q_hov_pk, 'chk','none', 'boxId',bi, ...
        'placed',placedBefore, 'convRemain',convRemainAfter, 'carrying',true);
    
    frameIdx=frameIdx+1;
    allPoses{frameIdx} = struct('name',sprintf('转向放箱%d',bi), ...
        'q',q_hov_pl, 'chk','none', 'boxId',bi, ...
        'placed',placedBefore, 'convRemain',convRemainAfter, 'carrying',true);
    
    frameIdx=frameIdx+1;
    allPoses{frameIdx} = struct('name',sprintf('放箱%d',bi), ...
        'q',q_release, 'chk','place', 'boxId',bi, ...
        'placed',placedBefore, 'convRemain',convRemainAfter, 'carrying',true);
    
    frameIdx=frameIdx+1;
    allPoses{frameIdx} = struct('name',sprintf('释放箱%d后退',bi), ...
        'q',q_hov_pl, 'chk','none', 'boxId',bi, ...
        'placed',placedAfter, 'convRemain',convRemainAfter, 'carrying',false);
end

frameIdx=frameIdx+1;
allPoses{frameIdx} = struct('name','任务完成', 'q', q_home, 'chk','none', ...
    'boxId',0, 'placed',nBoxes, 'convRemain',[], 'carrying',false);

N = length(allPoses);
fprintf('📋 总帧数: %d (3箱×6步 + 首尾)\n\n', N);

%% ====================== 主渲染循环 (v8.0 增强碰撞检测) ======================
res = cell(N,1);
collisionLog = [];   % 碰撞日志

for idx = 1:N
    p = allPoses{idx};
    q = p.q;
    
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('🤖 [%d/%d] %s  q=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]°\n', ...
        idx, N, p.name, rad2deg(q));
    
    fig = figure('Position', [50 50 1600 1000], 'Color', 'w');
    
    % ===== 左侧: 3D场景 (占主区域) =====
    ax_main = axes('Position', [0.02 0.05 0.68 0.88]); hold on;
    global alpha; alpha = 0.5;
    
    % 绘制静态场景
    drawGround(-1.5, 2.0, -2.0, 3.0);
    drawCabinet(cab, baseX, baseY);
    drawFrame_cage(frame);
    drawPallet(pallet, frame);
    drawConveyorBeltY(conv);
    
    % 传送带上剩余箱子
    bzp = convSurfZ + box.hz/2;
    for ci = 1:length(p.convRemain)
        bi2 = p.convRemain(ci);
        drawBox([convBoxX, convBoxY(bi2), bzp], box);
    end
    
    % 蓝框内已放好的箱子
    for pi2 = 1:p.placed
        pz_center = placePos(pi2,3) - box.hz/2;
        drawBox([placePos(pi2,1), placePos(pi2,2), pz_center], box);
    end
    
    % 绘制机器人
    [T00,T01,T02,T03,T04,T05,T0T] = FK_SSerial(q);
    Tf = {Tb*T00, Tb*T01, Tb*T02, Tb*T03, Tb*T04, Tb*T05, Tb*T0T};
    for i=1:7, plotframe(Tf{i}, 0.08, true); end
    outS = plotSelfCollisonModel(Tf, params, params_tool);
    
    % 携带中的箱子
    Tw = Tb*T0T;
    ep = Tw(1:3,4);
    if p.carrying
        drawBox([ep(1), ep(2), ep(3)-0.01], box);
    end
    
    ez = T0T(1:3,3);
    dev = acosd(max(-1,min(1,dot(ez,[0;0;-1]))));
    fprintf('   TCP: [%.3f,%.3f,%.3f]m  偏差=%.1f°\n', ep, dev);
    
    % TCP验证
    tcpOK = true;
    if strcmp(p.chk, 'pick')
        targetXY = [convBoxX, convBoxY(p.boxId)];
        targetZ = convBoxTopZ;
        gapXY = sqrt((ep(1)-targetXY(1))^2 + (ep(2)-targetXY(2))^2);
        gapZ = abs(ep(3) - targetZ);
        ok = gapZ < 0.03 && gapXY < 0.30;
        fprintf('   📦 取箱%d: 误差XY=%.1fmm Z=%.1fmm %s\n', ...
            p.boxId, gapXY*1000, gapZ*1000, iff(ok,'✅','❌'));
        tcpOK = ok;
    elseif strcmp(p.chk, 'place')
        targetXY = placePos(p.boxId, 1:2);
        targetZ = placePos(p.boxId, 3);
        gapXY = sqrt((ep(1)-targetXY(1))^2 + (ep(2)-targetXY(2))^2);
        gapZ = abs(ep(3) - targetZ);
        ok = gapZ < 0.04 && gapXY < 0.30;
        fprintf('   📦 放箱%d: 误差XY=%.1fmm Z=%.1fmm %s\n', ...
            p.boxId, gapXY*1000, gapZ*1000, iff(ok,'✅','❌'));
        tcpOK = ok;
    end
    
    % ★★★ v8.0: 三层碰撞检测 ★★★
    
    % === 第1层: 自碰撞检测 ===
    [sc, selfInfo, selfMinDist, selfPairDists] = selfCollCheck_v8(outS, params);
    
    % === 第2层: 环境碰撞检测 (完整版) ===
    [ec, envInfo, envMinDist, envDetailDists] = envCollCheck_v8(outS, params, envObjs, Tb);
    
    % === 第3层: TCP区域检测 ===
    tcpInFrame = isTCPInFrame(ep, frame);
    tcpOnGround = ep(3) < GROUND_Z + 0.02;
    tcpInfo = '';
    if tcpOnGround
        tcpInfo = 'TCP触地!';
    elseif tcpInFrame && ~strcmp(p.chk,'place') && ~strcmp(p.chk,'none')
        % 取箱时TCP不应在框架内
    end
    
    % === 综合碰撞判断 ===
    overallMinDist = min(selfMinDist, envMinDist);
    isCollision = sc || ec || tcpOnGround;
    isWarning   = ~isCollision && overallMinDist < COLL_THRESH_WARNING;
    isCaution   = ~isCollision && ~isWarning && overallMinDist < COLL_THRESH_CAUTION;
    
    if isCollision
        fprintf('   🔴 碰撞! '); tc=[0.85 0 0];
        if sc, fprintf('自碰撞: %s ', selfInfo); end
        if ec, fprintf('环境碰撞: %s ', envInfo); end
        if tcpOnGround, fprintf('TCP触地 '); end
        fprintf('\n');
    elseif isWarning
        fprintf('   🟡 警告 最小距离: %.3fm (自:%.3f 环:%.3f)\n', overallMinDist, selfMinDist, envMinDist);
        tc = [0.85 0.55 0];
    elseif isCaution
        fprintf('   🟠 注意 最小距离: %.3fm (自:%.3f 环:%.3f)\n', overallMinDist, selfMinDist, envMinDist);
        tc = [0.80 0.65 0.10];
    else
        fprintf('   🟢 安全 最小距离: %.3fm (自:%.3f 环:%.3f)\n', overallMinDist, selfMinDist, envMinDist);
        tc = [0 0.55 0];
    end
    
    % ★★★ v8.0: 碰撞体颜色渲染 (距离色阶) ★★★
    drawCollisionHeatmap(outS, params, selfPairDists, envDetailDists, ...
        COLL_THRESH_DANGER, COLL_THRESH_WARNING, COLL_THRESH_CAUTION);
    
    % ★★★ v8.0: 最小距离连线标注 ★★★
    if ~isempty(selfPairDists)
        [~, sIdx] = min([selfPairDists.dist]);
        sp = selfPairDists(sIdx);
        if sp.dist < COLL_THRESH_CAUTION
            plot3([sp.pt1(1) sp.pt2(1)], [sp.pt1(2) sp.pt2(2)], [sp.pt1(3) sp.pt2(3)], ...
                'r-', 'LineWidth', 2.5);
            mid = (sp.pt1 + sp.pt2) / 2;
            text(mid(1), mid(2), mid(3)+0.05, sprintf('%.0fmm', sp.dist*1000), ...
                'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r', ...
                'BackgroundColor', [1 1 0.8], 'HorizontalAlignment', 'center');
        end
    end
    
    if ~isempty(envDetailDists)
        [~, eIdx] = min([envDetailDists.dist]);
        ep2 = envDetailDists(eIdx);
        if ep2.dist < COLL_THRESH_CAUTION
            plot3([ep2.pt1(1) ep2.pt2(1)], [ep2.pt1(2) ep2.pt2(2)], [ep2.pt1(3) ep2.pt2(3)], ...
                'm-', 'LineWidth', 2.5);
            mid2 = (ep2.pt1 + ep2.pt2) / 2;
            text(mid2(1), mid2(2), mid2(3)+0.05, sprintf('%.0fmm', ep2.dist*1000), ...
                'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.6 0 0.6], ...
                'BackgroundColor', [1 0.9 1], 'HorizontalAlignment', 'center');
        end
    end
    
    % 标记取/放目标
    if strcmp(p.chk,'pick')
        plot3(convBoxX, convBoxY(p.boxId), convBoxTopZ, 'rv', 'MarkerSize',14, 'LineWidth',2);
        plot3(ep(1), ep(2), ep(3), 'g^', 'MarkerSize',14, 'LineWidth',2);
    elseif strcmp(p.chk,'place')
        plot3(placePos(p.boxId,1), placePos(p.boxId,2), placePos(p.boxId,3), 'rv','MarkerSize',14,'LineWidth',2);
        plot3(ep(1), ep(2), ep(3), 'g^', 'MarkerSize',14, 'LineWidth',2);
    end
    
    % 传送方向箭头
    arrY0 = conv.cy - conv.lengthY/2 + 0.15;
    arrY1 = conv.cy + conv.lengthY/2 - 0.15;
    arrZ = convSurfZ + conv.beltH + 0.08;
    plot3([conv.cx conv.cx], [arrY0 arrY1], [arrZ arrZ], 'm-', 'LineWidth', 1.5);
    plot3(conv.cx, arrY1, arrZ, 'm^', 'MarkerSize', 8, 'MarkerFaceColor','m');
    
    % 箱号标注
    for ci = 1:length(p.convRemain)
        bi2 = p.convRemain(ci);
        text(convBoxX, convBoxY(bi2), convSurfZ+box.hz+0.08, ...
            sprintf('#%d',bi2), 'FontSize',8, 'FontWeight','bold', ...
            'HorizontalAlignment','center', 'Color',[.8 .3 0]);
    end
    for pi2 = 1:p.placed
        text(placePos(pi2,1), placePos(pi2,2), placePos(pi2,3)+0.08, ...
            sprintf('#%d',pi2), 'FontSize',8, 'FontWeight','bold', ...
            'HorizontalAlignment','center', 'Color',[.0 .4 .7]);
    end
    
    title(sprintf('HR S50-2000 码垛 v8.0: [%d/%d] %s', idx,N,p.name), ...
        'FontSize', 13, 'Color', tc, 'FontWeight', 'bold');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on;
    xlim([-1.5 2.0]); ylim([-2.0 3.0]); zlim([0 2.5]);
    view(-45, 25);
    camlight('headlight'); lighting gouraud;
    
    % ===== 右侧: 碰撞检测仪表盘 (v8.0) =====
    ax_dash = axes('Position', [0.72 0.05 0.26 0.88]); hold on;
    axis off;
    drawCollisionDashboard(ax_dash, idx, N, p.name, ...
        sc, selfInfo, selfMinDist, selfPairDists, ...
        ec, envInfo, envMinDist, envDetailDists, ...
        tcpOnGround, ep, overallMinDist, isCollision, isWarning, isCaution, ...
        COLL_THRESH_WARNING, COLL_THRESH_CAUTION);
    
    drawnow;
    
    % 保存结果
    res{idx} = struct('name',p.name, 'ep',ep, 'dev',dev, ...
        'sc',sc, 'ec',ec, 'sd',selfMinDist, 'ed',envMinDist, ...
        'tcpOK',tcpOK, 'chk',p.chk, 'boxId',p.boxId, ...
        'overallMinDist', overallMinDist, 'isCollision', isCollision);
    
    collisionLog = [collisionLog; struct('frame',idx, 'selfDist',selfMinDist, ...
        'envDist',envMinDist, 'minDist',overallMinDist, ...
        'collision',isCollision, 'warning',isWarning)];
    
    if isHeadless
        fn = sprintf('%s/pose_%02d.png', outputDir, idx);
        saveas(fig, fn); close(fig);
        fprintf('   📁 %s\n', fn);
    end
end

%% ====================== 汇总 (v8.0) ======================
fprintf('\n╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║            码垛仿真结果汇总 v8.0 (三层碰撞检测增强版)                  ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║ 帧 │ 动作           │ 状态 │ 自碰撞   │ 环境碰撞 │ 最小距离  │ 附加   ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');

cc=0; wc=0; tcpAll=true;
for i=1:N
    r=res{i};
    if r.isCollision, s='🔴'; cc=cc+1;
    elseif r.overallMinDist<COLL_THRESH_WARNING, s='🟡'; wc=wc+1;
    elseif r.overallMinDist<COLL_THRESH_CAUTION, s='🟠';
    else, s='🟢';
    end
    if ~r.tcpOK, tcpAll=false; end
    extra='';
    if strcmp(r.chk,'pick')
        extra=sprintf('取#%d', r.boxId);
    elseif strcmp(r.chk,'place')
        extra=sprintf('放#%d', r.boxId);
    end
    fprintf('║ %2d │ %-14s │ %s  │ %7.3fm │ %7.3fm │ %7.3fm  │ %-6s ║\n', ...
        i, r.name, s, r.sd, r.ed, r.overallMinDist, extra);
end
fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║ 碰撞:%d  警告:%d  总帧:%d  TCP精度:%s  箱数:%d                     ║\n', ...
    cc, wc, N, iff(tcpAll,'✅','❌'), nBoxes);
fprintf('║ 检测体系: [1]连杆自碰撞(5体10对) [2]连杆-环境(%d物体) [3]TCP区域    ║\n', ...
    length(envObjs));
fprintf('╚══════════════════════════════════════════════════════════════════════════╝\n');

%% ====================== 碰撞距离趋势图 (v8.0) ======================
fig_trend = figure('Position', [100 100 1200 500], 'Color', 'w');

subplot(1,2,1);
selfDists = [collisionLog.selfDist];
envDists  = [collisionLog.envDist];
minDists  = [collisionLog.minDist];
frames    = 1:N;

hold on;
fill([frames fliplr(frames)], [zeros(1,N) ones(1,N)*COLL_THRESH_DANGER], [1 0.85 0.85], ...
    'EdgeColor','none', 'FaceAlpha', 0.3);
fill([frames fliplr(frames)], [ones(1,N)*COLL_THRESH_DANGER ones(1,N)*COLL_THRESH_WARNING], [1 1 0.8], ...
    'EdgeColor','none', 'FaceAlpha', 0.3);
fill([frames fliplr(frames)], [ones(1,N)*COLL_THRESH_WARNING ones(1,N)*COLL_THRESH_CAUTION], [1 0.95 0.8], ...
    'EdgeColor','none', 'FaceAlpha', 0.3);

plot(frames, selfDists, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
plot(frames, envDists, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
plot(frames, minDists, 'k-d', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor', 'y');

yline(COLL_THRESH_DANGER, 'r--', '碰撞', 'LineWidth', 1.5);
yline(COLL_THRESH_WARNING, 'Color', [0.8 0.6 0], 'LineStyle', '--', 'Label', '警告');
yline(COLL_THRESH_CAUTION, 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'Label', '注意');

legend('碰撞区', '警告区', '注意区', '自碰撞距离', '环境碰撞距离', '综合最小距离', ...
    'Location', 'best');
xlabel('帧序号'); ylabel('最小距离 (m)');
title('碰撞距离趋势 — 三层检测', 'FontSize', 12, 'FontWeight', 'bold');
grid on; xlim([0.5 N+0.5]);
set(gca, 'FontSize', 10);

% 右侧: 距离热力图
subplot(1,2,2);
linkNames = {'基座','下臂','肘部','上臂','腕部'};
heatData = zeros(5, N);
for idx2 = 1:N
    % 重新计算每连杆的环境最小距离
    p2 = allPoses{idx2};
    q2 = p2.q;
    [T00,T01,T02,T03,T04,T05,T0T] = FK_SSerial(q2);
    Tf2 = {Tb*T00, Tb*T01, Tb*T02, Tb*T03, Tb*T04, Tb*T05, Tb*T0T};
    outS2 = computeCollisionPoints(Tf2, params);
    
    linkSegs = getLinkSegments(outS2, params);
    for li = 1:5
        minD = inf;
        for oi = 1:length(envObjs)
            d = segAABBDist(linkSegs(li).p1, linkSegs(li).p2, linkSegs(li).r, ...
                envObjs(oi).aabbMin, envObjs(oi).aabbMax);
            if d < minD, minD = d; end
        end
        heatData(li, idx2) = minD;
    end
end

imagesc(heatData);
colormap(gca, customHeatmap());
colorbar;
caxis([0 0.3]);
set(gca, 'YTick', 1:5, 'YTickLabel', linkNames);
set(gca, 'XTick', 1:N);
xlabel('帧序号'); ylabel('连杆');
title('连杆-环境距离热力图 (m)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

if isHeadless
    saveas(fig_trend, sprintf('%s/collision_trend.png', outputDir));
    close(fig_trend);
    fprintf('\n📊 碰撞距离趋势图已保存\n');
end

%% ====================== GIF ======================
if isHeadless
    gifFile = sprintf('%s/palletizing_v8_anim.gif', outputDir);
    fprintf('\n🎬 生成GIF: %s\n', gifFile);
    for idx = 1:N
        fn = sprintf('%s/pose_%02d.png', outputDir, idx);
        if ~exist(fn,'file'), continue; end
        img = imread(fn);
        [A,map] = rgb2ind(img, 256);
        delay = 0.8;
        if idx==1 || idx==N, delay=1.5; end
        if contains(allPoses{idx}.name,'取箱') || contains(allPoses{idx}.name,'放箱')
            delay = 1.2;
        end
        if idx == 1
            imwrite(A, map, gifFile, 'gif', 'LoopCount', 0, 'DelayTime', delay);
        else
            imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
        end
    end
    fprintf('✅ GIF完成 (%d帧)\n', N);
end

fprintf('\n✅ v8.0 码垛仿真完成! (三层碰撞检测增强版)\n');

%% ========================================================================
%%                     v8.0 增强碰撞检测函数
%% ========================================================================

%% ==================== 构建环境碰撞模型 ====================
function objs = buildEnvironmentModel(cab, frame, pallet, conv, bx, by, groundZ)
    objs = struct('name',{}, 'aabbMin',{}, 'aabbMax',{}, 'type',{});
    
    % 1. 地面
    objs(end+1).name = '地面';
    objs(end).aabbMin = [-2, -3, groundZ-0.02];
    objs(end).aabbMax = [3, 4, groundZ];
    objs(end).type = 'ground';
    
    % 2. 电箱
    objs(end+1).name = '电箱';
    objs(end).aabbMin = [bx-cab.widthX/2, by-cab.depthY/2, 0];
    objs(end).aabbMax = [bx+cab.widthX/2, by+cab.depthY/2, cab.heightZ];
    objs(end).type = 'box';
    
    % 3-6. 框架立柱
    wx = frame.widthX; dy = frame.depthY; cx = frame.cx; cy = frame.cy;
    corners = [cx-wx/2 cy-dy/2; cx+wx/2 cy-dy/2; cx+wx/2 cy+dy/2; cx-wx/2 cy+dy/2];
    pillarNames = {'框柱-左前','框柱-右前','框柱-右后','框柱-左后'};
    for i = 1:4
        objs(end+1).name = pillarNames{i};
        objs(end).aabbMin = [corners(i,1)-frame.tubeR, corners(i,2)-frame.tubeR, 0];
        objs(end).aabbMax = [corners(i,1)+frame.tubeR, corners(i,2)+frame.tubeR, frame.height];
        objs(end).type = 'pillar';
    end
    
    % 7-9. 框架横梁 (非开口面, 即+X, +Y, -X面)
    beamNames = {'+X面横梁','+Y面横梁','-X面横梁'};
    beamEdges = {[2,3], [3,4], [4,1]};  % 跳过1-2 (开口-Y面)
    for bi = 1:3
        e = beamEdges{bi};
        i1 = e(1); i2 = e(2);
        objs(end+1).name = beamNames{bi};
        x1 = min(corners(i1,1), corners(i2,1)); x2 = max(corners(i1,1), corners(i2,1));
        y1 = min(corners(i1,2), corners(i2,2)); y2 = max(corners(i1,2), corners(i2,2));
        objs(end).aabbMin = [x1-frame.tubeR*0.8, y1-frame.tubeR*0.8, 0];
        objs(end).aabbMax = [x2+frame.tubeR*0.8, y2+frame.tubeR*0.8, frame.height];
        objs(end).type = 'beam';
    end
    
    % 10. 托盘
    objs(end+1).name = '托盘';
    objs(end).aabbMin = [frame.cx-pallet.widthX/2, frame.cy-pallet.depthY/2, 0];
    objs(end).aabbMax = [frame.cx+pallet.widthX/2, frame.cy+pallet.depthY/2, pallet.heightZ];
    objs(end).type = 'box';
    
    % 11. 传送带
    objs(end+1).name = '传送带';
    objs(end).aabbMin = [conv.cx-conv.widthX/2, conv.cy-conv.lengthY/2, 0];
    objs(end).aabbMax = [conv.cx+conv.widthX/2, conv.cy+conv.lengthY/2, conv.heightZ+conv.rollerR+conv.beltH];
    objs(end).type = 'box';
end

%% ==================== v8.0: 增强自碰撞检测 ====================
function [ic, info, minDist, pairDists] = selfCollCheck_v8(outS, p)
    ic = false; info = ''; minDist = inf;
    pairDists = struct('name',{}, 'dist',{}, 'pt1',{}, 'pt2',{}, 'linkA',{}, 'linkB',{});
    
    pairs = {
        {'基座',  '肘部',  outS.base_bc1(:), outS.base_bc2(:), p.base.radius, ...
                          outS.elbow_e1(:), outS.elbow_e2(:), p.elbow.radius};
        {'基座',  '上臂',  outS.base_bc1(:), outS.base_bc2(:), p.base.radius, ...
                          outS.upperArm_ua1(:), outS.upperArm_ua2(:), p.upperArm.radius};
        {'基座',  '腕部',  outS.base_bc1(:), outS.base_bc2(:), p.base.radius, ...
                          outS.wrist_wc(:), outS.wrist_wc(:), p.wrist.radius};
        {'下臂',  '上臂',  outS.lowerArm_la1(:), outS.lowerArm_la2(:), p.lowerArm.radius, ...
                          outS.upperArm_ua1(:), outS.upperArm_ua2(:), p.upperArm.radius};
        {'下臂',  '腕部',  outS.lowerArm_la1(:), outS.lowerArm_la2(:), p.lowerArm.radius, ...
                          outS.wrist_wc(:), outS.wrist_wc(:), p.wrist.radius};
        {'肘部',  '腕部',  outS.elbow_e1(:), outS.elbow_e2(:), p.elbow.radius, ...
                          outS.wrist_wc(:), outS.wrist_wc(:), p.wrist.radius};
        {'基座',  '下臂',  outS.base_bc1(:), outS.base_bc2(:), p.base.radius, ...
                          outS.lowerArm_la1(:), outS.lowerArm_la2(:), p.lowerArm.radius};
        {'肘部',  '上臂',  outS.elbow_e1(:), outS.elbow_e2(:), p.elbow.radius, ...
                          outS.upperArm_ua1(:), outS.upperArm_ua2(:), p.upperArm.radius};
        {'上臂',  '腕部',  outS.upperArm_ua1(:), outS.upperArm_ua2(:), p.upperArm.radius, ...
                          outS.wrist_wc(:), outS.wrist_wc(:), p.wrist.radius};
    };
    
    for i = 1:length(pairs)
        pr = pairs{i};
        nameA = pr{1}; nameB = pr{2};
        p1 = pr{3}; p2 = pr{4}; r1 = pr{5};
        p3 = pr{6}; p4 = pr{7}; r2 = pr{8};
        
        [d, cp1, cp2] = segSegDist_v8(p1, p2, p3, p4);
        netDist = d - r1 - r2;
        
        pairDists(end+1).name = sprintf('%s-%s', nameA, nameB);
        pairDists(end).dist = netDist;
        % 最近点(在胶囊体表面)
        if d > 1e-6
            dir = (cp2 - cp1) / d;
        else
            dir = [0;0;1];
        end
        pairDists(end).pt1 = cp1 + dir * r1;
        pairDists(end).pt2 = cp2 - dir * r2;
        pairDists(end).linkA = nameA;
        pairDists(end).linkB = nameB;
        
        if netDist < minDist
            minDist = netDist;
        end
        
        if netDist < 0 && ~ic
            ic = true;
            info = sprintf('%s-%s(%.3fm)', nameA, nameB, netDist);
        end
    end
end

%% ==================== v8.0: 增强环境碰撞检测 ====================
function [ic, info, minDist, detailDists] = envCollCheck_v8(outS, p, envObjs, Tb)
    ic = false; info = ''; minDist = inf;
    detailDists = struct('name',{}, 'dist',{}, 'pt1',{}, 'pt2',{}, 'link',{}, 'obj',{});
    
    % 所有连杆
    links = {
        {'基座',  outS.base_bc1(:), outS.base_bc2(:), p.base.radius};
        {'下臂',  outS.lowerArm_la1(:), outS.lowerArm_la2(:), p.lowerArm.radius};
        {'肘部',  outS.elbow_e1(:), outS.elbow_e2(:), p.elbow.radius};
        {'上臂',  outS.upperArm_ua1(:), outS.upperArm_ua2(:), p.upperArm.radius};
        {'腕部',  outS.wrist_wc(:), outS.wrist_wc(:), p.wrist.radius};
    };
    
    for li = 1:length(links)
        lk = links{li};
        linkName = lk{1};
        lp1 = lk{2}; lp2 = lk{3}; lr = lk{4};
        
        for oi = 1:length(envObjs)
            obj = envObjs(oi);
            
            % 基座跳过与地面和电箱的检测(它们是连接的)
            if strcmp(linkName, '基座') && (strcmp(obj.type,'ground') || strcmp(obj.name,'电箱'))
                continue;
            end
            
            % 计算连杆胶囊体到AABB的距离
            [d, cp_link, cp_box] = segAABBDistFull(lp1, lp2, lr, obj.aabbMin(:), obj.aabbMax(:));
            
            detailDists(end+1).name = sprintf('%s→%s', linkName, obj.name);
            detailDists(end).dist = d;
            detailDists(end).pt1 = cp_link;
            detailDists(end).pt2 = cp_box;
            detailDists(end).link = linkName;
            detailDists(end).obj = obj.name;
            
            if d < minDist
                minDist = d;
            end
            
            if d < 0 && ~ic
                ic = true;
                info = sprintf('%s→%s(%.3fm)', linkName, obj.name, d);
            end
        end
    end
end

%% ==================== TCP在框架内检测 ====================
function inFrame = isTCPInFrame(tcpPos, frame)
    cx = frame.cx; cy = frame.cy;
    wx = frame.widthX/2; dy = frame.depthY/2;
    inFrame = tcpPos(1) > cx-wx && tcpPos(1) < cx+wx && ...
              tcpPos(2) > cy-dy && tcpPos(2) < cy+dy && ...
              tcpPos(3) < frame.height;
end

%% ==================== 碰撞距离色阶渲染 ====================
function drawCollisionHeatmap(outS, params, selfPairDists, envDetailDists, ...
    threshDanger, threshWarn, threshCaution)
    
    % 计算每个连杆的最小距离
    linkNames = {'基座','下臂','肘部','上臂','腕部'};
    linkMinDist = inf(1, 5);
    
    % 从自碰撞中提取
    for i = 1:length(selfPairDists)
        sp = selfPairDists(i);
        for li = 1:5
            if contains(sp.name, linkNames{li})
                if sp.dist < linkMinDist(li)
                    linkMinDist(li) = sp.dist;
                end
            end
        end
    end
    
    % 从环境碰撞中提取
    for i = 1:length(envDetailDists)
        ed = envDetailDists(i);
        for li = 1:5
            if strcmp(ed.link, linkNames{li})
                if ed.dist < linkMinDist(li)
                    linkMinDist(li) = ed.dist;
                end
            end
        end
    end
    
    % 绘制半透明碰撞体轮廓, 颜色按距离
    linkSegs = {
        {outS.base_bc1(:), outS.base_bc2(:), params.base.radius};
        {outS.lowerArm_la1(:), outS.lowerArm_la2(:), params.lowerArm.radius};
        {outS.elbow_e1(:), outS.elbow_e2(:), params.elbow.radius};
        {outS.upperArm_ua1(:), outS.upperArm_ua2(:), params.upperArm.radius};
        {outS.wrist_wc(:), outS.wrist_wc(:), params.wrist.radius};
    };
    
    for li = 1:5
        d = linkMinDist(li);
        % 距离到颜色映射
        if d <= threshDanger
            col = [1.0, 0.0, 0.0];  % 红: 碰撞
            alp = 0.5;
        elseif d <= threshWarn
            t = d / threshWarn;
            col = [1.0, t*0.8, 0.0];  % 红→橙: 警告
            alp = 0.35;
        elseif d <= threshCaution
            t = (d - threshWarn) / (threshCaution - threshWarn);
            col = [1.0-t*0.5, 0.8+t*0.2, t*0.2];  % 橙→黄绿: 注意
            alp = 0.25;
        else
            col = [0.2, 0.85, 0.3];  % 绿: 安全
            alp = 0.15;
        end
        
        seg = linkSegs{li};
        drawTransparentCapsule(seg{1}, seg{2}, seg{3}*1.05, col, alp);
    end
end

%% ==================== 绘制半透明胶囊体 ====================
function drawTransparentCapsule(p1, p2, r, color, alpha_val)
    if norm(p2-p1) < 1e-6
        % 球体
        [X,Y,Z] = sphere(12);
        surf(X*r+p1(1), Y*r+p1(2), Z*r+p1(3), ...
            'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha_val);
        return;
    end
    
    % 柱体部分
    v = p2 - p1;
    L = norm(v);
    [X,Y,Z] = cylinder(r, 12);
    Z = Z * L;
    
    dd = [0;0;1]; td = v/L;
    cp = cross(dd, td);
    if norm(cp) > 1e-6
        ax = cp/norm(cp);
        ag = acos(max(-1,min(1,dot(dd,td))));
        R = axang2r_v8([ax', ag]);
    else
        R = eye(3);
        if dot(dd,td) < 0, R(3,3)=-1; R(1,1)=-1; end
    end
    
    for i = 1:numel(X)
        pt = R*[X(i);Y(i);Z(i)];
        X(i) = pt(1) + p1(1);
        Y(i) = pt(2) + p1(2);
        Z(i) = pt(3) + p1(3);
    end
    surf(X,Y,Z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha_val);
    
    % 端球
    [SX,SY,SZ] = sphere(8);
    surf(SX*r+p1(1), SY*r+p1(2), SZ*r+p1(3), ...
        'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha_val);
    surf(SX*r+p2(1), SY*r+p2(2), SZ*r+p2(3), ...
        'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha_val);
end

%% ==================== 碰撞仪表盘 ====================
function drawCollisionDashboard(ax, idx, N, poseName, ...
    sc, selfInfo, selfMinDist, selfPairDists, ...
    ec, envInfo, envMinDist, envDetailDists, ...
    tcpOnGround, tcpPos, overallMinDist, isCollision, isWarning, isCaution, ...
    threshWarn, threshCaution)
    
    axes(ax);
    
    % 背景
    rectangle('Position', [0 0 1 1], 'FaceColor', [0.97 0.97 0.97], ...
        'EdgeColor', [0.7 0.7 0.7], 'LineWidth', 1.5, 'Curvature', 0.02);
    
    yPos = 0.96;
    
    % 标题
    text(0.5, yPos, '🛡️ 碰撞检测仪表盘', 'FontSize', 12, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'Color', [0.2 0.2 0.3]);
    yPos = yPos - 0.04;
    
    text(0.5, yPos, sprintf('[%d/%d] %s', idx, N, poseName), 'FontSize', 9, ...
        'HorizontalAlignment', 'center', 'Color', [0.4 0.4 0.4]);
    yPos = yPos - 0.05;
    
    % === 综合状态 ===
    if isCollision
        statusColor = [0.9 0.1 0.1]; statusText = '🔴 碰撞';
        statusBg = [1 0.9 0.9];
    elseif isWarning
        statusColor = [0.85 0.55 0]; statusText = '🟡 警告';
        statusBg = [1 1 0.88];
    elseif isCaution
        statusColor = [0.7 0.5 0.1]; statusText = '🟠 注意';
        statusBg = [1 0.97 0.88];
    else
        statusColor = [0 0.6 0.2]; statusText = '🟢 安全';
        statusBg = [0.9 1 0.9];
    end
    
    rectangle('Position', [0.05 yPos-0.04 0.9 0.06], 'FaceColor', statusBg, ...
        'EdgeColor', statusColor, 'LineWidth', 2, 'Curvature', 0.3);
    text(0.5, yPos-0.01, sprintf('%s  最小距离: %.1fmm', statusText, overallMinDist*1000), ...
        'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
        'Color', statusColor);
    yPos = yPos - 0.07;
    
    % === 第1层: 自碰撞 ===
    text(0.05, yPos, '━━━ 第1层: 连杆自碰撞 ━━━', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.2 0.2 0.6]);
    yPos = yPos - 0.03;
    
    if sc
        text(0.08, yPos, sprintf('🔴 %s', selfInfo), 'FontSize', 8, 'Color', [0.8 0 0]);
    else
        text(0.08, yPos, sprintf('✅ 最小: %.1fmm', selfMinDist*1000), ...
            'FontSize', 8, 'Color', [0 0.5 0]);
    end
    yPos = yPos - 0.025;
    
    % 显示前5个最近的自碰撞对
    if ~isempty(selfPairDists)
        dists = [selfPairDists.dist];
        [~, sortIdx] = sort(dists);
        nShow = min(5, length(sortIdx));
        for i = 1:nShow
            sp = selfPairDists(sortIdx(i));
            if sp.dist < 0
                c = [0.8 0 0]; marker = '●';
            elseif sp.dist < threshWarn
                c = [0.8 0.5 0]; marker = '◉';
            else
                c = [0.3 0.6 0.3]; marker = '○';
            end
            text(0.08, yPos, sprintf('%s %-10s %6.1fmm', marker, sp.name, sp.dist*1000), ...
                'FontSize', 7, 'FontName', 'FixedWidth', 'Color', c);
            yPos = yPos - 0.02;
        end
    end
    yPos = yPos - 0.01;
    
    % === 第2层: 环境碰撞 ===
    text(0.05, yPos, '━━━ 第2层: 连杆-环境碰撞 ━━━', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.5 0.2 0.2]);
    yPos = yPos - 0.03;
    
    if ec
        text(0.08, yPos, sprintf('🔴 %s', envInfo), 'FontSize', 8, 'Color', [0.8 0 0]);
    else
        text(0.08, yPos, sprintf('✅ 最小: %.1fmm', envMinDist*1000), ...
            'FontSize', 8, 'Color', [0 0.5 0]);
    end
    yPos = yPos - 0.025;
    
    % 显示前6个最近的环境碰撞对
    if ~isempty(envDetailDists)
        dists = [envDetailDists.dist];
        [~, sortIdx] = sort(dists);
        nShow = min(6, length(sortIdx));
        for i = 1:nShow
            ed = envDetailDists(sortIdx(i));
            if ed.dist < 0
                c = [0.8 0 0]; marker = '●';
            elseif ed.dist < threshWarn
                c = [0.8 0.5 0]; marker = '◉';
            elseif ed.dist < threshCaution
                c = [0.6 0.5 0.1]; marker = '◎';
            else
                c = [0.3 0.6 0.3]; marker = '○';
            end
            text(0.08, yPos, sprintf('%s %-18s %6.1fmm', marker, ed.name, ed.dist*1000), ...
                'FontSize', 7, 'FontName', 'FixedWidth', 'Color', c);
            yPos = yPos - 0.02;
        end
    end
    yPos = yPos - 0.01;
    
    % === 第3层: TCP区域 ===
    text(0.05, yPos, '━━━ 第3层: TCP状态 ━━━', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.2 0.5 0.2]);
    yPos = yPos - 0.03;
    
    text(0.08, yPos, sprintf('TCP: [%.3f, %.3f, %.3f]m', tcpPos(1), tcpPos(2), tcpPos(3)), ...
        'FontSize', 8, 'Color', [0.3 0.3 0.3]);
    yPos = yPos - 0.02;
    
    if tcpOnGround
        text(0.08, yPos, '🔴 TCP触地!', 'FontSize', 8, 'Color', [0.8 0 0]);
    else
        text(0.08, yPos, sprintf('✅ TCP高度: %.0fmm', tcpPos(3)*1000), ...
            'FontSize', 8, 'Color', [0 0.5 0]);
    end
    yPos = yPos - 0.03;
    
    % === 距离条形图 ===
    text(0.05, yPos, '━━━ 连杆安全距离条 ━━━', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.3 0.3 0.5]);
    yPos = yPos - 0.02;
    
    linkNames = {'基座','下臂','肘部','上臂','腕部'};
    linkMinDist = inf(1, 5);
    
    % 综合自碰撞和环境碰撞的最小距离
    for i = 1:length(selfPairDists)
        sp = selfPairDists(i);
        for li = 1:5
            if contains(sp.name, linkNames{li}) && sp.dist < linkMinDist(li)
                linkMinDist(li) = sp.dist;
            end
        end
    end
    for i = 1:length(envDetailDists)
        ed = envDetailDists(i);
        for li = 1:5
            if strcmp(ed.link, linkNames{li}) && ed.dist < linkMinDist(li)
                linkMinDist(li) = ed.dist;
            end
        end
    end
    
    maxBar = 0.3;  % 最大显示距离 300mm
    barH = 0.018;
    for li = 1:5
        d = min(linkMinDist(li), maxBar);
        barW = max(0.01, (d / maxBar) * 0.7);
        
        if linkMinDist(li) <= 0
            barColor = [0.9 0.15 0.15];
        elseif linkMinDist(li) < threshWarn
            barColor = [0.9 0.6 0.1];
        elseif linkMinDist(li) < threshCaution
            barColor = [0.8 0.8 0.2];
        else
            barColor = [0.2 0.75 0.3];
        end
        
        % 背景条
        rectangle('Position', [0.22 yPos-barH/2 0.7 barH], 'FaceColor', [0.92 0.92 0.92], ...
            'EdgeColor', 'none');
        % 实际距离条
        rectangle('Position', [0.22 yPos-barH/2 barW barH], 'FaceColor', barColor, ...
            'EdgeColor', 'none', 'Curvature', 0.3);
        
        text(0.05, yPos, linkNames{li}, 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
        
        if linkMinDist(li) < inf
            text(0.93, yPos, sprintf('%.0f', linkMinDist(li)*1000), 'FontSize', 7, ...
                'HorizontalAlignment', 'right', 'Color', barColor, 'FontWeight', 'bold');
        else
            text(0.93, yPos, 'N/A', 'FontSize', 7, 'HorizontalAlignment', 'right', ...
                'Color', [0.6 0.6 0.6]);
        end
        
        yPos = yPos - 0.03;
    end
    
    xlim([0 1]); ylim([0 1]);
end

%% ==================== 几何工具 (v8.0增强) ====================

% 线段-线段距离 (返回最近点)
function [d, cp1, cp2] = segSegDist_v8(p1, p2, p3, p4)
    d1 = p2 - p1; d2 = p4 - p3; r = p1 - p3;
    a = dot(d1,d1); b = dot(d1,d2); c = dot(d2,d2);
    dd = dot(d1,r); e = dot(d2,r);
    dn = a*c - b*b;
    
    if dn < 1e-10
        s = 0; t = dd / max(b, 1e-10);
    else
        s = (b*e - c*dd) / dn;
        t = (a*e - b*dd) / dn;
    end
    s = max(0, min(1, s));
    t = max(0, min(1, t));
    
    cp1 = p1 + s*d1;
    cp2 = p3 + t*d2;
    d = norm(cp1 - cp2);
end

% 点到AABB的距离
function [d, cp] = ptAABBDist(pt, aabbMin, aabbMax)
    cp = max(aabbMin, min(pt, aabbMax));
    d = norm(pt - cp);
end

% 线段到AABB的距离
function d = segAABBDist(p1, p2, radius, aabbMin, aabbMax)
    nSamples = 10;
    minD = inf;
    for t = linspace(0, 1, nSamples)
        pt = p1 + t*(p2 - p1);
        [d2, ~] = ptAABBDist(pt(:), aabbMin(:), aabbMax(:));
        netD = d2 - radius;
        if netD < minD, minD = netD; end
    end
    d = minD;
end

% 线段到AABB的距离 (返回最近点对)
function [d, cpLink, cpBox] = segAABBDistFull(p1, p2, radius, aabbMin, aabbMax)
    nSamples = 15;
    minD = inf;
    bestT = 0;
    bestBoxPt = p1;
    
    for t = linspace(0, 1, nSamples)
        pt = p1 + t*(p2 - p1);
        [d2, cp] = ptAABBDist(pt(:), aabbMin(:), aabbMax(:));
        netD = d2 - radius;
        if netD < minD
            minD = netD;
            bestT = t;
            bestBoxPt = cp;
        end
    end
    
    % 精细搜索
    for t = linspace(max(0,bestT-0.1), min(1,bestT+0.1), 20)
        pt = p1 + t*(p2 - p1);
        [d2, cp] = ptAABBDist(pt(:), aabbMin(:), aabbMax(:));
        netD = d2 - radius;
        if netD < minD
            minD = netD;
            bestT = t;
            bestBoxPt = cp;
        end
    end
    
    linkPt = p1 + bestT*(p2 - p1);
    d = minD;
    
    if norm(linkPt - bestBoxPt) > 1e-8
        dir = (bestBoxPt - linkPt) / norm(bestBoxPt - linkPt);
        cpLink = linkPt + dir * radius;
    else
        cpLink = linkPt;
    end
    cpBox = bestBoxPt;
end

% 获取连杆线段(用于热力图)
function linkSegs = getLinkSegments(outS, params)
    linkSegs = struct('p1',{}, 'p2',{}, 'r',{});
    
    linkSegs(1).p1 = outS.base_bc1(:);
    linkSegs(1).p2 = outS.base_bc2(:);
    linkSegs(1).r  = params.base.radius;
    
    linkSegs(2).p1 = outS.lowerArm_la1(:);
    linkSegs(2).p2 = outS.lowerArm_la2(:);
    linkSegs(2).r  = params.lowerArm.radius;
    
    linkSegs(3).p1 = outS.elbow_e1(:);
    linkSegs(3).p2 = outS.elbow_e2(:);
    linkSegs(3).r  = params.elbow.radius;
    
    linkSegs(4).p1 = outS.upperArm_ua1(:);
    linkSegs(4).p2 = outS.upperArm_ua2(:);
    linkSegs(4).r  = params.upperArm.radius;
    
    linkSegs(5).p1 = outS.wrist_wc(:);
    linkSegs(5).p2 = outS.wrist_wc(:);
    linkSegs(5).r  = params.wrist.radius;
end

% 计算碰撞体端点 (不绘图, 用于热力图)
function outS = computeCollisionPoints(Tf_tree, params)
    outS = struct();
    T00 = Tf_tree{1}; T02 = Tf_tree{3}; T03 = Tf_tree{4};
    T04 = Tf_tree{5}; T05 = Tf_tree{6};
    
    % 基座胶囊体 (使用动态字段名规避 end 关键字)
    bs = params.base.start(:);
    be = params.base.('end');  be = be(:);
    outS.base_bc1 = T00(1:3,1:3)*bs + T00(1:3,4);
    outS.base_bc2 = T00(1:3,1:3)*be + T00(1:3,4);
    
    % 下臂
    las = params.lowerArm.start(:);
    lae = params.lowerArm.('end');  lae = lae(:);
    outS.lowerArm_la1 = T02(1:3,1:3)*las + T02(1:3,4);
    outS.lowerArm_la2 = T02(1:3,1:3)*lae + T02(1:3,4);
    
    % 肘部
    es = params.elbow.start(:);
    ee = params.elbow.('end');  ee = ee(:);
    outS.elbow_e1 = T03(1:3,1:3)*es + T03(1:3,4);
    outS.elbow_e2 = T03(1:3,1:3)*ee + T03(1:3,4);
    
    % 上臂
    uas = params.upperArm.start(:);
    uae = params.upperArm.('end');  uae = uae(:);
    outS.upperArm_ua1 = T04(1:3,1:3)*uas + T04(1:3,4);
    outS.upperArm_ua2 = T04(1:3,1:3)*uae + T04(1:3,4);
    
    % 腕部
    outS.wrist_wc = T05(1:3,1:3)*params.wrist.offset(:) + T05(1:3,4);
end

% 自定义热力图颜色
function cmap = customHeatmap()
    n = 256;
    cmap = zeros(n, 3);
    for i = 1:n
        t = (i-1)/(n-1);
        if t < 0.15
            cmap(i,:) = [0.8, 0, 0];           % 深红: 碰撞
        elseif t < 0.3
            s = (t-0.15)/0.15;
            cmap(i,:) = [1, s*0.6, 0];          % 红→橙
        elseif t < 0.5
            s = (t-0.3)/0.2;
            cmap(i,:) = [1-s*0.2, 0.6+s*0.3, s*0.1]; % 橙→黄
        elseif t < 0.7
            s = (t-0.5)/0.2;
            cmap(i,:) = [0.8-s*0.6, 0.9, 0.1+s*0.4]; % 黄→绿
        else
            s = (t-0.7)/0.3;
            cmap(i,:) = [0.2-s*0.1, 0.9-s*0.2, 0.5+s*0.3]; % 绿→青
        end
    end
end

%% ==================== IK搜索 ====================
function [q2_best, q3_best, q4_best, err_best] = searchIK_ext(q1, target_r, target_z)
    global d1 d2 d3 d4 d5 d6 a2 a3 T6T;
    err_best = inf;
    q2_best=-pi/2; q3_best=pi/2; q4_best=-pi/2;
    
    for q2d = -175:3:-30
        for q3d = 10:3:165
            for q4d = [-150,-135,-120,-90,-60,-45]
                q2=q2d*pi/180; q3=q3d*pi/180; q4=q4d*pi/180;
                [~,~,~,~,~,~,T] = FK_SSerial([q1,q2,q3,q4,-pi/2,0]);
                r_fk=sqrt(T(1,4)^2+T(2,4)^2); z_fk=T(3,4);
                pe=sqrt((r_fk-target_r)^2+(z_fk-target_z)^2);
                ez=T(1:3,3); zd=acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                tot=pe+0.08*zd;
                if tot<err_best && zd<30*pi/180
                    err_best=pe; q2_best=q2; q3_best=q3; q4_best=q4;
                end
            end
        end
    end
    
    q2c=rad2deg(q2_best); q3c=rad2deg(q3_best); q4c=rad2deg(q4_best);
    for q2d=(q2c-5):0.5:(q2c+5)
        for q3d=(q3c-5):0.5:(q3c+5)
            for q4d=(q4c-10):2:(q4c+10)
                if q2d<-175||q2d>-30||q3d<10||q3d>165, continue; end
                q2=q2d*pi/180; q3=q3d*pi/180; q4=q4d*pi/180;
                [~,~,~,~,~,~,T] = FK_SSerial([q1,q2,q3,q4,-pi/2,0]);
                r_fk=sqrt(T(1,4)^2+T(2,4)^2); z_fk=T(3,4);
                pe=sqrt((r_fk-target_r)^2+(z_fk-target_z)^2);
                ez=T(1:3,3); zd=acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                tot=pe+0.08*zd;
                if tot<err_best && zd<30*pi/180
                    err_best=pe; q2_best=q2; q3_best=q3; q4_best=q4;
                end
            end
        end
    end
end

%% ==================== 基础绘图函数 ====================
function s = iff(c,a,b)
    if c, s=a; else, s=b; end
end

function drawGround(x0,x1,y0,y1)
    patch('Vertices',[x0 y0 0;x1 y0 0;x1 y1 0;x0 y1 0],'Faces',[1 2 3 4],...
          'FaceColor',[.92 .92 .90],'EdgeColor','none','FaceAlpha',0.4);
end

function drawCabinet(c, bx, by)
    x=bx-c.widthX/2; y=by-c.depthY/2; w=c.widthX; d=c.depthY; h=c.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',c.color,'EdgeColor',[.5 .5 .5],'FaceAlpha',.95,'LineWidth',1);
    sv=[x+.06 y+d-.01 h*.55; x+w-.06 y+d-.01 h*.55; x+w-.06 y+d-.01 h*.80; x+.06 y+d-.01 h*.80];
    patch('Vertices',sv,'Faces',[1 2 3 4],'FaceColor',[.6 .9 .6],'EdgeColor','k','LineWidth',1.5);
    [sx,sy,sz]=sphere(10); r=.025;
    surf(sx*r+bx, sy*r+by+d/2-.01, sz*r+h*.38, 'FaceColor',[.9 .1 .1],'EdgeColor','none');
end

function drawFrame_cage(f)
    r=f.tubeR; wx=f.widthX; dy=f.depthY; h=f.height;
    cx=f.cx; cy=f.cy;
    c=[cx-wx/2 cy-dy/2; cx+wx/2 cy-dy/2; cx+wx/2 cy+dy/2; cx-wx/2 cy+dy/2];
    for i=1:4, drawTube(c(i,1),c(i,2),0,c(i,1),c(i,2),h,r,f.color); end
    edges={[1,2],[2,3],[3,4],[4,1]};
    openEdge=1;
    for hz=[0.05 h/3 2*h/3 h-0.05]
        for ei=1:4
            if ei==openEdge, continue; end
            i1=edges{ei}(1); i2=edges{ei}(2);
            drawTube(c(i1,1),c(i1,2),hz,c(i2,1),c(i2,2),hz,r*.8,f.color);
        end
    end
    for ei=1:4
        if ei==openEdge, continue; end
        i1=edges{ei}(1); i2=edges{ei}(2);
        x1=c(i1,1); y1=c(i1,2); x2=c(i2,1); y2=c(i2,2);
        for k=0:7
            z1=k*h/8+0.05; z2=z1+h/16;
            mx=(x1+x2)/2; my=(y1+y2)/2;
            drawTube(x1,y1,z1,mx,my,z2,r*.18,f.color*.85);
            drawTube(mx,my,z2,x2,y2,z1+h/8-0.05,r*.18,f.color*.85);
        end
    end
end

function drawPallet(pal, frm)
    x=frm.cx-pal.widthX/2; y=frm.cy-pal.depthY/2;
    w=pal.widthX; d=pal.depthY; h=pal.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',pal.color,'EdgeColor',[.15 .30 .60],'FaceAlpha',.90,'LineWidth',1.2);
    text(frm.cx, frm.cy, h+0.03, sprintf('蓝色托盘 %dcm', round(h*100)), 'FontSize',7, ...
         'HorizontalAlignment','center', 'Color',[.05 .15 .45], 'FontWeight','bold');
end

function drawConveyorBeltY(cv)
    cx=cv.cx; cy=cv.cy; ly=cv.lengthY; wx=cv.widthX; hz=cv.heightZ;
    x0=cx-wx/2; y0=cy-ly/2;
    drawBox3D(x0-.015,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    drawBox3D(x0+wx,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    lw=.035; yL=[y0+.2 cy y0+ly-.2];
    for yi=1:3
        for s=[-1 1]
            lx=cx+s*(wx/2-.06);
            drawBox3D(lx-lw/2,yL(yi)-lw/2,0,lw,lw,hz-.01,[.25 .25 .25]);
        end
        drawBox3D(x0+.04,yL(yi)-lw/2,hz*.35,wx-.08,lw,lw,[.25 .25 .25]);
    end
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        drawRollerX(cx,ry,hz,wx*.9,cv.rollerR,[.55 .55 .55]);
    end
    bz=hz+cv.rollerR;
    bV=[x0+.02 y0+.04 bz;x0+wx-.02 y0+.04 bz;x0+wx-.02 y0+ly-.04 bz;x0+.02 y0+ly-.04 bz;
        x0+.02 y0+.04 bz+cv.beltH;x0+wx-.02 y0+.04 bz+cv.beltH;
        x0+wx-.02 y0+ly-.04 bz+cv.beltH;x0+.02 y0+ly-.04 bz+cv.beltH];
    patch('Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[.15 .15 .15],'FaceAlpha',.92);
    sZ=bz+cv.beltH+.002;
    patch('Vertices',[cx-.012 y0+.06 sZ;cx+.012 y0+.06 sZ;cx+.012 y0+ly-.06 sZ;cx-.012 y0+ly-.06 sZ],...
          'Faces',[1 2 3 4],'FaceColor',[.9 .8 .2],'EdgeColor','none');
    [mx,my,mz]=cylinder(.05,10); mz=mz*.10;
    surf(mx+x0-.08,my+y0+ly-.15,mz+hz-.02,'FaceColor',[.3 .5 .3],'EdgeColor','none','FaceAlpha',.85);
end

function drawRollerX(cx,y,z,w,r,col)
    [X,Y,Z]=cylinder(r,8); Z=Z*w-w/2;
    surf(Z+cx,zeros(size(X))+y,X+z+r,'FaceColor',col,'EdgeColor','none','FaceAlpha',.6);
end

function drawBox3D(x,y,z,dx,dy,dz,col)
    v=[x y z;x+dx y z;x+dx y+dy z;x y+dy z;x y z+dz;x+dx y z+dz;x+dx y+dy z+dz;x y+dy z+dz];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',col,'EdgeColor','none','FaceAlpha',.95);
end

function drawBox(pos,bx)
    x=pos(1)-bx.lx/2; y=pos(2)-bx.wy/2; z=pos(3)-bx.hz/2;
    v=[x y z;x+bx.lx y z;x+bx.lx y+bx.wy z;x y+bx.wy z;
       x y z+bx.hz;x+bx.lx y z+bx.hz;x+bx.lx y+bx.wy z+bx.hz;x y+bx.wy z+bx.hz];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',bx.color,'EdgeColor',[.35 .25 .1],'FaceAlpha',.95,'LineWidth',1.2);
    text(pos(1),pos(2),pos(3)+bx.hz/2+.01,'BOX','FontSize',6,'HorizontalAlignment','center','Color',[.2 .1 .05]);
end

function drawTube(x1,y1,z1,x2,y2,z2,radius,color)
    [X,Y,Z]=cylinder(radius,10);
    v=[x2-x1;y2-y1;z2-z1]; l=norm(v);
    if l<.001, return; end
    Z=Z*l;
    dd=[0;0;1]; td=v/l; cp=cross(dd,td);
    if norm(cp)>1e-6
        ax=cp/norm(cp); ag=acos(max(-1,min(1,dot(dd,td))));
        R=axang2r_v8([ax',ag]);
    else
        R=eye(3);
        if dot(dd,td)<0, R(3,3)=-1; R(1,1)=-1; end
    end
    for i=1:numel(X)
        pt=R*[X(i);Y(i);Z(i)]; X(i)=pt(1)+x1; Y(i)=pt(2)+y1; Z(i)=pt(3)+z1;
    end
    surf(X,Y,Z,'FaceColor',color,'EdgeColor','none','FaceAlpha',.85);
end

function R=axang2r_v8(ax)
    a=ax(1:3); g=ax(4);
    c=cos(g); s=sin(g); t=1-c;
    x=a(1); y=a(2); z=a(3);
    R=[t*x*x+c t*x*y-s*z t*x*z+s*y;t*x*y+s*z t*y*y+c t*y*z-s*x;t*x*z-s*y t*y*z+s*x t*z*z+c];
end
