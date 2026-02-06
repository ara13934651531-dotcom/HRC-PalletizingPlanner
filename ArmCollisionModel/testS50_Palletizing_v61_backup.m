%% testS50_Palletizing.m - HR_S50-2000 码垛工作站场景仿真 (v6.1)
%
% 对照实际场景照片配置:
%   - 电箱高度≈0.80m (白色控制柜, 绿色屏幕, 红色急停)
%   - 蓝色框架(笼式)在机械臂前方(+Y方向, x=0)
%   - 传送带/箱子台在机械臂右侧(+X方向)
%   - 框架开口朝-Y方向, 正对电箱和机械臂
%   - 各组件之间有充足间距, 绝无重叠
%
% 布局 (俯视图):
%
%        +Y (前方)
%        ^
%        |   ┌──────────┐
%        |   │  蓝色框架  │
%        |   └────开口────┘  (开口朝-Y, 正对机械臂)
%        |        |
%        |     [电箱+机械臂] ────── ══传送带══
%        |      (0,0)               (+X方向)
%   -----+------------------------------------> +X
%
% 腕部角度: q4=-135°可将工作空间下探到z_base≈0.26m(r≈0.73m)
%   和z_base≈0.05m(r≈1.03m), 满足cab=0.80m时取放箱需求
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

close all; clear all; clc;
addpath('collisionVisual'); addpath(genpath('collisionVisual'));

%% ====================== 环境设置 ======================
isHeadless = ~usejava('desktop');
outputDir = './pic/S50_palletizing';
if isHeadless, set(0, 'DefaultFigureVisible', 'off'); end
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

%% ====================== 场景几何参数 ======================
% 单位: 米(m), 对照实际照片配置

% ── 电箱/控制柜 (匹配实际照片~0.80m高) ──
cab.widthX  = 0.55;
cab.depthY  = 0.65;
cab.heightZ = 0.80;   % ★ 匹配实际照片高度
cab.color   = [0.95, 0.95, 0.93];

% ── 蓝色笼式框架 (在机械臂前方+Y, 开口朝-Y正对机械臂) ──
frame.widthX  = 1.20;   % X方向宽度
frame.depthY  = 1.00;   % Y方向深度
frame.height  = 2.00;   % 高度
frame.tubeR   = 0.030;  % 管材半径
frame.color   = [0.25, 0.55, 0.85];

% ── 框内码垛托盘 ──
pallet.widthX = 0.90;
pallet.depthY = 0.70;
pallet.heightZ = 0.60;  % 托盘高度 (箱顶0.60+0.25=0.85m)
pallet.color  = [0.70, 0.58, 0.40];

% ── 传送带 (在机械臂右侧+X, 长度沿Y) ──
% 实际照片: 箱子台在机械臂右手边
conv.lengthY  = 2.00;   % 沿Y方向长度
conv.widthX   = 0.55;   % X方向宽度
conv.heightZ  = 0.75;   % 皮带面高度
conv.beltH    = 0.035;
conv.rollerR  = 0.030;
conv.nRollers = 12;
conv.color    = [0.30, 0.30, 0.32];

% ── 箱子 ──
box.lx = 0.40; box.wy = 0.30; box.hz = 0.25;
box.color = [0.65, 0.45, 0.25];

%% ====================== 布局 (符合实际照片) ======================
baseZ = cab.heightZ;  % 0.80m

% 箱子顶面高度
convBoxSurfZ = conv.heightZ + conv.rollerR + conv.beltH + box.hz;
% = 0.75 + 0.03 + 0.035 + 0.25 = 1.065m
palletBoxSurfZ = pallet.heightZ + box.hz;
% = 0.60 + 0.25 = 0.85m

% 基座在原点 (电箱中心)
baseX = 0.0; baseY = 0.0;

% 蓝框在前方 (+Y方向), x=0居中, 开口朝-Y正对机械臂
frameGap = 0.40;  % 框架近边缘(-Y边)到电箱远边缘(+Y边)间距
frame.cx = 0.0;   % ★ x=0 居中
frame.cy = cab.depthY/2 + frameGap + frame.depthY/2;  % +Y方向
% 框架开口朝-Y (面向机械臂)
frame.openDir = '-Y';  % 标记开口方向

% 传送带在右侧 (+X方向), 留足间距
convGap = 0.40;  % 传送带边缘到电箱边缘间距
conv.cx = cab.widthX/2 + convGap + conv.widthX/2;
conv.cy = 0.0;   % Y方向与基座对齐

fprintf('╔════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  HR_S50-2000 码垛工作站 v6.1 - 蓝框+Y方向, 开口朝-Y正对机械臂      ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════╝\n\n');

fprintf('📐 布局 (间距检查):\n');
fprintf('   电箱: 中心(%.2f,%.2f), 尺寸%.2f×%.2f×%.2fm\n', baseX, baseY, cab.widthX, cab.depthY, cab.heightZ);
fprintf('   蓝框: 中心(%.2f,%.2f), 开口朝-Y (正对机械臂)\n', frame.cx, frame.cy);
fprintf('   传送带: 中心(%.2f,%.2f), 长度沿Y\n', conv.cx, conv.cy);

% 间距验证
% 蓝框-Y边 vs 电箱+Y边
cabFront = baseY + cab.depthY/2;
frameBack = frame.cy - frame.depthY/2;  % 框架-Y边(开口侧)
gap_cab_frame = frameBack - cabFront;   % 蓝框开口到电箱前沿间距

% 传送带vs电箱
cabRight = baseX + cab.widthX/2;
convLeft = conv.cx - conv.widthX/2;
gap_cab_conv = convLeft - cabRight;

% 蓝框vs传送带 (对角间距)
frameSE_x = frame.cx + frame.widthX/2;  % 框架右下角
frameSE_y = frame.cy - frame.depthY/2;
gap_frame_conv = sqrt((convLeft - frameSE_x)^2 + max(0, frameSE_y - conv.cy - conv.lengthY/2)^2);

fprintf('   电箱+Y边→蓝框-Y边(开口)间距: %.3fm\n', gap_cab_frame);
fprintf('   电箱右边→传送带左边间距: %.3fm\n', gap_cab_conv);
fprintf('   蓝框角→传送带间距: %.3fm\n', gap_frame_conv);
assert(gap_cab_frame >= 0.30, '电箱与蓝框间距不足!');
assert(gap_cab_conv >= 0.30, '电箱与传送带间距不足!');
fprintf('   ✅ 间距充足, 无重叠\n\n');

% 目标位置
pickWorld = [conv.cx, baseY, convBoxSurfZ];
placeWorld = [frame.cx, frame.cy, palletBoxSurfZ];

fprintf('📍 目标位置:\n');
fprintf('   取箱: [%.3f, %.3f, %.3f] (传送带箱顶)\n', pickWorld);
fprintf('   放箱: [%.3f, %.3f, %.3f] (蓝框内托盘箱顶)\n\n', placeWorld);

%% ====================== 加载碰撞模型 ======================
params = readCollisionModelJson("./model/collideConfig/S50_collision.json");
params_tool = readToolCollisionJson("./model/collideConfig/nonetool_collision.json");
fprintf('✅ 碰撞模型加载完成\n\n');

%% ====================== FK全局变量 ======================
global d1 d2 d3 d4 d5 d6 a2 a3 T6T;
d1=params.DH.d1; d2=params.DH.d2; d3=params.DH.d3;
d4=params.DH.d4; d5=params.DH.d5; d6=params.DH.d6;
a2=-params.DH.a2; a3=-params.DH.a3; T6T=eye(4);

%% ====================== 数值IK求解 ======================
% J1方向: q1 = atan2(-dy, -dx)
% 使用q4=-135°扩展低z可达范围

% 取箱方向 (+X)
pick_dx = pickWorld(1) - baseX;
pick_dy = pickWorld(2) - baseY;
q1_pick = atan2(-pick_dy, -pick_dx);

% 放箱方向 (-X)
place_dx = placeWorld(1) - baseX;
place_dy = placeWorld(2) - baseY;
q1_place = atan2(-place_dy, -place_dx);

fprintf('📍 J1方向:\n');
fprintf('   取箱: dx=%.3f dy=%.3f → J1=%.1f°\n', pick_dx, pick_dy, rad2deg(q1_pick));
fprintf('   放箱: dx=%.3f dy=%.3f → J1=%.1f°\n\n', place_dx, place_dy, rad2deg(q1_place));

% IK搜索
target_r_pick = sqrt(pick_dx^2 + pick_dy^2);
target_z_pick = convBoxSurfZ - baseZ;  % 1.065-0.80 = 0.265m

target_r_place = sqrt(place_dx^2 + place_dy^2);
target_z_place = palletBoxSurfZ - baseZ;  % 0.85-0.80 = 0.05m

fprintf('🔍 IK搜索 (扩展腕部q4)...\n');
fprintf('   取箱目标: r=%.3fm, z=%.3fm (基座系)\n', target_r_pick, target_z_pick);
fprintf('   放箱目标: r=%.3fm, z=%.3fm (基座系)\n\n', target_r_place, target_z_place);

[q2_pk, q3_pk, q4_pk, err_pk] = searchIK_ext(q1_pick, target_r_pick, target_z_pick);
fprintf('   取箱IK: q2=%.1f° q3=%.1f° q4=%.1f° 误差=%.1fmm\n', ...
    rad2deg(q2_pk), rad2deg(q3_pk), rad2deg(q4_pk), err_pk*1000);

[q2_pl, q3_pl, q4_pl, err_pl] = searchIK_ext(q1_place, target_r_place, target_z_place);
fprintf('   放箱IK: q2=%.1f° q3=%.1f° q4=%.1f° 误差=%.1fmm\n\n', ...
    rad2deg(q2_pl), rad2deg(q3_pl), rad2deg(q4_pl), err_pl*1000);

% FK验证
q_pk_test = [q1_pick, q2_pk, q3_pk, q4_pk, -pi/2, 0];
[~,~,~,~,~,~,T_pk] = FK_SSerial(q_pk_test);
Tb = eye(4); Tb(1,4)=baseX; Tb(2,4)=baseY; Tb(3,4)=baseZ;
tcp_pk = Tb * [T_pk(1:3,4); 1];

q_pl_test = [q1_place, q2_pl, q3_pl, q4_pl, -pi/2, 0];
[~,~,~,~,~,~,T_pl] = FK_SSerial(q_pl_test);
tcp_pl = Tb * [T_pl(1:3,4); 1];

fprintf('🔬 FK验证 (世界坐标):\n');
fprintf('   取箱TCP: [%.3f, %.3f, %.3f] 目标z=%.3f 误差=%.1fmm\n', ...
    tcp_pk(1:3), convBoxSurfZ, abs(tcp_pk(3)-convBoxSurfZ)*1000);
fprintf('   放箱TCP: [%.3f, %.3f, %.3f] 目标z=%.3f 误差=%.1fmm\n\n', ...
    tcp_pl(1:3), palletBoxSurfZ, abs(tcp_pl(3)-palletBoxSurfZ)*1000);

% TCP实际XY用于对齐箱子绘制
pickBoxXY = tcp_pk(1:2)';
placeBoxXY = tcp_pl(1:2)';

%% ====================== 姿态序列 ======================
q_home = [q1_pick, -pi/3, pi/4, 0, -pi/3, 0];

% 悬停用折叠姿态 (r较小, 远离障碍)
q_hover_pick  = [q1_pick,  -pi/2, 2*pi/3, -pi/2, -pi/2, 0];
q_hover_place = [q1_place, -pi/2, 2*pi/3, -pi/2, -pi/2, 0];

q_grasp   = [q1_pick,  q2_pk, q3_pk, q4_pk, -pi/2, 0];
q_release = [q1_place, q2_pl, q3_pl, q4_pl, -pi/2, 0];

poses = {
    struct('name','初始待机',  'q',q_home,        'chk','none');
    struct('name','传送带悬停','q',q_hover_pick,   'chk','none');
    struct('name','下降取箱',  'q',q_grasp,        'chk','pick');
    struct('name','提升箱子',  'q',q_hover_pick,   'chk','none');
    struct('name','转向码垛',  'q',q_hover_place,  'chk','none');
    struct('name','码垛悬停',  'q',q_hover_place,  'chk','none');
    struct('name','下降放置',  'q',q_release,       'chk','place');
    struct('name','释放提升',  'q',q_hover_place,  'chk','none');
    struct('name','返回传送带','q',q_hover_pick,    'chk','none');
    struct('name','回到初始',  'q',q_home,          'chk','none');
};

%% ====================== 主循环 ======================
N = length(poses);
res = cell(N,1);

for idx = 1:N
    p = poses{idx};
    q = p.q;
    
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('🤖 %d/%d: %s  q=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]°\n', ...
        idx, N, p.name, rad2deg(q));
    
    fig = figure('Position', [50 50 1400 1000], 'Color', 'w'); hold on;
    global alpha; alpha = 0.5;
    
    % --- 绘制场景 ---
    drawGround(-1.5, 2.0, -1.5, 2.5);
    drawCabinet(cab, baseX, baseY);
    drawFrame_cage(frame);  % 笼式蓝框
    drawPallet(pallet, frame);
    drawConveyorBeltY(conv);  % 沿Y方向的传送带
    
    % 传送带上的箱子 (第1个对齐TCP)
    bzp = conv.heightZ + conv.rollerR + conv.beltH + box.hz/2;
    drawBox([pickBoxXY(1), pickBoxXY(2), bzp], box);
    for bi = 2:3
        byp = pickBoxXY(2) + (bi-1)*0.50;
        drawBox([conv.cx, byp, bzp], box);
    end
    
    % 蓝框内箱子
    if idx >= 8
        drawBox([placeBoxXY(1), placeBoxXY(2), palletBoxSurfZ - box.hz/2], box);
    end
    
    % --- 绘制机器人 ---
    [T00,T01,T02,T03,T04,T05,T0T] = FK_SSerial(q);
    Tf = {Tb*T00, Tb*T01, Tb*T02, Tb*T03, Tb*T04, Tb*T05, Tb*T0T};
    for i=1:7, plotframe(Tf{i}, 0.08, true); end
    outS = plotSelfCollisonModel(Tf, params, params_tool);
    
    % 末端位置
    Tw = Tb*T0T;
    ep = Tw(1:3,4);
    ez = T0T(1:3,3);
    dev = acosd(max(-1,min(1,dot(ez,[0;0;-1]))));
    
    fprintf('   末端(世界): [%.3f, %.3f, %.3f]m  方向偏差=%.1f°\n', ep, dev);
    
    % TCP贴合验证
    tcpOK = true;
    if strcmp(p.chk, 'pick')
        gap = abs(ep(3) - convBoxSurfZ);
        ok = gap < 0.03;
        fprintf('   📦 取箱TCP: 箱顶=%.3fm TCP=%.3fm 误差=%.1fmm %s\n', ...
            convBoxSurfZ, ep(3), gap*1000, iff(ok,'✅','❌'));
        tcpOK = ok;
    elseif strcmp(p.chk, 'place')
        gap = abs(ep(3) - palletBoxSurfZ);
        ok = gap < 0.04;
        fprintf('   📦 放箱TCP: 箱顶=%.3fm TCP=%.3fm 误差=%.1fmm %s\n', ...
            palletBoxSurfZ, ep(3), gap*1000, iff(ok,'✅','❌'));
        tcpOK = ok;
    end
    
    % 碰撞检测
    [sc,si,sd] = selfCollCheck(outS, params);
    [ec,ei,ed] = envCollCheck(outS, params, frame, conv, cab, baseX, baseY);
    
    if sc
        fprintf('   ⚠️ 自碰撞: %s\n', si); tc=[.8 0 0];
    elseif ec
        fprintf('   ⚠️ 环境碰撞: %s\n', ei); tc=[.8 .4 0];
    else
        fprintf('   ✅ 安全 (自:%.3f 环:%.3f)\n', sd, ed); tc=[0 .5 0];
    end
    
    res{idx} = struct('name',p.name,'ep',ep,'dev',dev,'sc',sc,'ec',ec,'sd',sd,'ed',ed,'tcpOK',tcpOK,'chk',p.chk);
    
    % TCP标记
    if strcmp(p.chk, 'pick')
        plot3(pickBoxXY(1), pickBoxXY(2), convBoxSurfZ, 'rv', 'MarkerSize', 12, 'LineWidth', 2);
        plot3(ep(1), ep(2), ep(3), 'g^', 'MarkerSize', 12, 'LineWidth', 2);
    elseif strcmp(p.chk, 'place')
        plot3(placeBoxXY(1), placeBoxXY(2), palletBoxSurfZ, 'rv', 'MarkerSize', 12, 'LineWidth', 2);
        plot3(ep(1), ep(2), ep(3), 'g^', 'MarkerSize', 12, 'LineWidth', 2);
    end
    
    title(sprintf('HR S50-2000 码垛 v6.1: %s', p.name), 'FontSize',14, 'Color',tc, 'FontWeight','bold');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on;
    xlim([-1.5 2.0]); ylim([-1.5 2.5]); zlim([0 2.5]);
    view(-45, 25);
    camlight('headlight'); lighting gouraud;
    drawnow;
    
    if isHeadless
        fn = sprintf('%s/pose_%02d.png', outputDir, idx);
        saveas(fig, fn); close(fig);
        fprintf('   📁 %s\n', fn);
    end
end

%% ====================== 汇总 ======================
fprintf('\n╔════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                   码垛仿真结果汇总 v6.1                            ║\n');
fprintf('╠════════════════════════════════════════════════════════════════════╣\n');
cc=0; tcpAll=true;
for i=1:N
    r=res{i};
    if r.sc||r.ec, s='⚠️碰'; cc=cc+1; else, s='✅安'; end
    if ~r.tcpOK, tcpAll=false; end
    extra = '';
    if strcmp(r.chk,'pick'), extra = sprintf(' TCP误差:%.0fmm', abs(r.ep(3)-convBoxSurfZ)*1000);
    elseif strcmp(r.chk,'place'), extra = sprintf(' TCP误差:%.0fmm', abs(r.ep(3)-palletBoxSurfZ)*1000);
    end
    fprintf('║ %2d. %-10s %s 偏差:%3.0f° TCP:[%+.2f,%+.2f,%.2f]%s\n', i, r.name, s, r.dev, r.ep, extra);
end
fprintf('╠════════════════════════════════════════════════════════════════════╣\n');
fprintf('║ 碰撞:%d/%d  TCP贴合:%s  电箱高:%.2fm               ║\n', ...
    cc, N, iff(tcpAll,'✅','❌'), cab.heightZ);
fprintf('║ 蓝框(0,+Y):%.2f,%.2f  传送带(+X):%.2f  框距:%.2f/带距:%.2fm  ║\n', ...
    frame.cx, frame.cy, conv.cx, gap_cab_frame, gap_cab_conv);
fprintf('╚════════════════════════════════════════════════════════════════════╝\n');

%% ====================== GIF ======================
if isHeadless
    gifFile = sprintf('%s/palletizing_anim.gif', outputDir);
    fprintf('\n🎬 生成GIF: %s\n', gifFile);
    for idx = 1:N
        fn = sprintf('%s/pose_%02d.png', outputDir, idx);
        img = imread(fn);
        [A,map] = rgb2ind(img, 256);
        if idx == 1
            imwrite(A, map, gifFile, 'gif', 'LoopCount', 0, 'DelayTime', 1.2);
        else
            imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 1.0);
        end
    end
    fprintf('✅ GIF生成完成\n');
end

fprintf('\n✅ 码垛仿真完成!\n');

%% ==================== 扩展IK搜索 (含q4变化) ====================
function [q2_best, q3_best, q4_best, err_best] = searchIK_ext(q1, target_r, target_z)
    global d1 d2 d3 d4 d5 d6 a2 a3 T6T;
    err_best = inf;
    q2_best = -pi/2; q3_best = pi/2; q4_best = -pi/2;
    
    % 第1遍: 粗搜索 (q4也搜)
    for q2d = -175:3:-30
        for q3d = 10:3:165
            for q4d = [-150, -135, -120, -90, -60, -45]
                q2=q2d*pi/180; q3=q3d*pi/180; q4=q4d*pi/180;
                q = [q1, q2, q3, q4, -pi/2, 0];
                [~,~,~,~,~,~,T] = FK_SSerial(q);
                
                r_fk = sqrt(T(1,4)^2 + T(2,4)^2);
                z_fk = T(3,4);
                pos_err = sqrt((r_fk-target_r)^2 + (z_fk-target_z)^2);
                
                ez = T(1:3,3);
                zdev = acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                
                total = pos_err + 0.08*zdev;
                if total < err_best && zdev < 30*pi/180
                    err_best = pos_err;
                    q2_best=q2; q3_best=q3; q4_best=q4;
                end
            end
        end
    end
    
    % 第2遍: 精细搜索
    q2c=rad2deg(q2_best); q3c=rad2deg(q3_best); q4c=rad2deg(q4_best);
    for q2d = (q2c-5):0.5:(q2c+5)
        for q3d = (q3c-5):0.5:(q3c+5)
            for q4d = (q4c-10):2:(q4c+10)
                if q2d<-175||q2d>-30||q3d<10||q3d>165, continue; end
                q2=q2d*pi/180; q3=q3d*pi/180; q4=q4d*pi/180;
                q = [q1, q2, q3, q4, -pi/2, 0];
                [~,~,~,~,~,~,T] = FK_SSerial(q);
                
                r_fk = sqrt(T(1,4)^2 + T(2,4)^2);
                z_fk = T(3,4);
                pos_err = sqrt((r_fk-target_r)^2 + (z_fk-target_z)^2);
                
                ez = T(1:3,3);
                zdev = acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                
                total = pos_err + 0.08*zdev;
                if total < err_best && zdev < 30*pi/180
                    err_best = pos_err;
                    q2_best=q2; q3_best=q3; q4_best=q4;
                end
            end
        end
    end
end

%% ==================== 辅助函数 ====================
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
    % 显示屏 (正面 +Y)
    sv=[x+.06 y+d-.01 h*.55; x+w-.06 y+d-.01 h*.55; x+w-.06 y+d-.01 h*.80; x+.06 y+d-.01 h*.80];
    patch('Vertices',sv,'Faces',[1 2 3 4],'FaceColor',[.6 .9 .6],'EdgeColor','k','LineWidth',1.5);
    % 急停按钮
    [sx,sy,sz]=sphere(10); r=.025;
    surf(sx*r+bx, sy*r+by+d/2-.01, sz*r+h*.38, 'FaceColor',[.9 .1 .1],'EdgeColor','none');
    % 文字
    text(bx, by+d/2+0.01, h*.35, '主电源开关', 'FontSize',5, 'HorizontalAlignment','center', 'Color',[.3 .3 .3]);
end

function drawFrame_cage(f)
    r=f.tubeR; wx=f.widthX; dy=f.depthY; h=f.height;
    cx=f.cx; cy=f.cy;
    c=[cx-wx/2 cy-dy/2; cx+wx/2 cy-dy/2; cx+wx/2 cy+dy/2; cx-wx/2 cy+dy/2];
    
    % 4根立柱
    for i=1:4, drawTube(c(i,1),c(i,2),0,c(i,1),c(i,2),h,r,f.color); end
    
    % 角点: 1=左下(-X,-Y), 2=右下(+X,-Y), 3=右上(+X,+Y), 4=左上(-X,+Y)
    % 边: 1→2(-Y底), 2→3(+X右), 3→4(+Y顶), 4→1(-X左)
    % 开口面 = 边1→2 (-Y侧, 朝向机械臂)
    edges = {[1,2],[2,3],[3,4],[4,1]};
    openEdge = 1;  % 跳过第1条边(-Y面开口, 朝向机械臂)
    
    % 横梁
    for hz=[0.05 h/3 2*h/3 h-0.05]
        for ei=1:4
            if ei == openEdge, continue; end
            i1=edges{ei}(1); i2=edges{ei}(2);
            drawTube(c(i1,1),c(i1,2),hz,c(i2,1),c(i2,2),hz,r*.8,f.color);
        end
    end
    
    % 网格面板 (模拟实际照片的菱形网格)
    for ei=1:4
        if ei == openEdge, continue; end
        i1=edges{ei}(1); i2=edges{ei}(2);
        x1=c(i1,1); y1=c(i1,2); x2=c(i2,1); y2=c(i2,2);
        nSeg=8;
        for k=0:nSeg-1
            z1 = k*h/nSeg+0.05; z2 = z1+h/(2*nSeg);
            mx=(x1+x2)/2; my=(y1+y2)/2;
            drawTube(x1,y1,z1,mx,my,z2,r*.18,f.color*.85);
            drawTube(mx,my,z2,x2,y2,z1+h/nSeg-0.05,r*.18,f.color*.85);
        end
    end
end

function drawPallet(pal, frm)
    x = frm.cx - pal.widthX/2;
    y = frm.cy - pal.depthY/2;
    w = pal.widthX; d = pal.depthY; h = pal.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',pal.color,'EdgeColor',[.45 .35 .2],'FaceAlpha',.85,'LineWidth',0.8);
    text(frm.cx, frm.cy, h+0.03, sprintf('托盘 %.2fm', h), 'FontSize',6, ...
         'HorizontalAlignment','center', 'Color',[.3 .2 .1]);
end

function drawConveyorBeltY(cv)
    cx=cv.cx; cy=cv.cy; ly=cv.lengthY; wx=cv.widthX; hz=cv.heightZ;
    x0=cx-wx/2; y0=cy-ly/2;
    
    % 侧板
    drawBox3D(x0-.015,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    drawBox3D(x0+wx,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    
    % 支架腿
    lw=.035; yL=[y0+.2 cy y0+ly-.2];
    for yi=1:3
        for s=[-1 1]
            lx=cx+s*(wx/2-.06);
            drawBox3D(lx-lw/2,yL(yi)-lw/2,0,lw,lw,hz-.01,[.25 .25 .25]);
        end
        drawBox3D(x0+.04,yL(yi)-lw/2,hz*.35,wx-.08,lw,lw,[.25 .25 .25]);
    end
    
    % 滚筒 (沿X方向)
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        drawRollerX(cx,ry,hz,wx*.9,cv.rollerR,[.55 .55 .55]);
    end
    
    % 皮带
    bz=hz+cv.rollerR;
    bV=[x0+.02 y0+.04 bz;x0+wx-.02 y0+.04 bz;x0+wx-.02 y0+ly-.04 bz;x0+.02 y0+ly-.04 bz;
        x0+.02 y0+.04 bz+cv.beltH;x0+wx-.02 y0+.04 bz+cv.beltH;
        x0+wx-.02 y0+ly-.04 bz+cv.beltH;x0+.02 y0+ly-.04 bz+cv.beltH];
    patch('Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[.15 .15 .15],'FaceAlpha',.92);
    
    % 中心线
    sZ=bz+cv.beltH+.002;
    patch('Vertices',[cx-.012 y0+.06 sZ;cx+.012 y0+.06 sZ;cx+.012 y0+ly-.06 sZ;cx-.012 y0+ly-.06 sZ],...
          'Faces',[1 2 3 4],'FaceColor',[.9 .8 .2],'EdgeColor','none');
    
    % 电机
    [mx,my,mz]=cylinder(.05,10); mz=mz*.10;
    surf(mx+x0-.08,my+y0+ly-.15,mz+hz-.02,'FaceColor',[.3 .5 .3],'EdgeColor','none','FaceAlpha',.85);
end

function drawRollerX(cx,y,z,w,r,col)
    [X,Y,Z]=cylinder(r,8); Z=Z*w-w/2;
    % 滚筒沿X方向
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
        R=axang2r([ax',ag]);
    else
        R=eye(3);
        if dot(dd,td)<0, R(3,3)=-1; R(1,1)=-1; end
    end
    for i=1:numel(X)
        pt=R*[X(i);Y(i);Z(i)]; X(i)=pt(1)+x1; Y(i)=pt(2)+y1; Z(i)=pt(3)+z1;
    end
    surf(X,Y,Z,'FaceColor',color,'EdgeColor','none','FaceAlpha',.85);
end

function [ic,info,md] = selfCollCheck(outS,p)
    ic=false; info=''; md=inf;
    pairs={
        {'基座-肘',outS.base_bc1(:),outS.base_bc2(:),p.base.radius,outS.elbow_e1(:),outS.elbow_e2(:),p.elbow.radius};
        {'基座-上臂',outS.base_bc1(:),outS.base_bc2(:),p.base.radius,outS.upperArm_ua1(:),outS.upperArm_ua2(:),p.upperArm.radius};
        {'基座-腕',outS.base_bc1(:),outS.base_bc2(:),p.base.radius,outS.wrist_wc(:),outS.wrist_wc(:),p.wrist.radius};
        {'下臂-上臂',outS.lowerArm_la1(:),outS.lowerArm_la2(:),p.lowerArm.radius,outS.upperArm_ua1(:),outS.upperArm_ua2(:),p.upperArm.radius};
        {'下臂-腕',outS.lowerArm_la1(:),outS.lowerArm_la2(:),p.lowerArm.radius,outS.wrist_wc(:),outS.wrist_wc(:),p.wrist.radius};
    };
    for i=1:length(pairs)
        pr=pairs{i};
        if norm(pr{5}-pr{6})<1e-6, d=ptSegD(pr{5},pr{2},pr{3});
        else, d=segSegD(pr{2},pr{3},pr{5},pr{6}); end
        n=d-pr{4}-pr{7};
        if n<md, md=n; end
        if n<0, ic=true; info=sprintf('%s(%.3f)',pr{1},n); return; end
    end
end

function [ic,info,md] = envCollCheck(outS,p,frm,cv,cab,bx,by)
    ic=false; info=''; md=inf;
    wx=frm.widthX; dy=frm.depthY; cx=frm.cx; cy=frm.cy;
    corners=[cx-wx/2 cy-dy/2;cx+wx/2 cy-dy/2;cx+wx/2 cy+dy/2;cx-wx/2 cy+dy/2];
    segs={
        {'下臂',outS.lowerArm_la1(:),outS.lowerArm_la2(:),p.lowerArm.radius};
        {'肘部',outS.elbow_e1(:),outS.elbow_e2(:),p.elbow.radius};
        {'上臂',outS.upperArm_ua1(:),outS.upperArm_ua2(:),p.upperArm.radius};
    };
    % 框架立柱
    for ci=1:4
        cp1=[corners(ci,1);corners(ci,2);0]; cp2=[corners(ci,1);corners(ci,2);frm.height];
        for ai=1:length(segs)
            sg=segs{ai};
            d=segSegD(sg{2},sg{3},cp1,cp2);
            n=d-sg{4}-frm.tubeR;
            if n<md, md=n; end
            if n<0, ic=true; info=sprintf('%s-框柱%d(%.3f)',sg{1},ci,n); return; end
        end
    end
    % 传送带侧板
    cvL = cv.cx - cv.widthX/2; cvR = cv.cx + cv.widthX/2;
    cvB = cv.cy - cv.lengthY/2; cvF = cv.cy + cv.lengthY/2;
    cvCorners = [cvL cvB; cvR cvB; cvR cvF; cvL cvF];
    for ci=1:4
        cp1=[cvCorners(ci,1);cvCorners(ci,2);0]; cp2=[cvCorners(ci,1);cvCorners(ci,2);cv.heightZ+0.05];
        for ai=1:length(segs)
            sg=segs{ai};
            d=segSegD(sg{2},sg{3},cp1,cp2);
            n=d-sg{4}-0.02;
            if n<md, md=n; end
            if n<0, ic=true; info=sprintf('%s-传送带(%.3f)',sg{1},n); return; end
        end
    end
end

function d=ptSegD(p,a,b)
    ab=b-a; ap=p-a; t=max(0,min(1,dot(ap,ab)/max(dot(ab,ab),1e-10)));
    d=norm(p-(a+t*ab));
end

function d=segSegD(p1,p2,p3,p4)
    d1=p2-p1; d2=p4-p3; r=p1-p3;
    a=dot(d1,d1); b=dot(d1,d2); c=dot(d2,d2); dd=dot(d1,r); e=dot(d2,r);
    dn=a*c-b*b;
    if dn<1e-10, s=0; t=dd/max(b,1e-10);
    else, s=(b*e-c*dd)/dn; t=(a*e-b*dd)/dn; end
    s=max(0,min(1,s)); t=max(0,min(1,t));
    d=norm((p1+s*d1)-(p3+t*d2));
end

function R=axang2r(ax)
    a=ax(1:3); g=ax(4);
    c=cos(g); s=sin(g); t=1-c;
    x=a(1); y=a(2); z=a(3);
    R=[t*x*x+c t*x*y-s*z t*x*z+s*y;t*x*y+s*z t*y*y+c t*y*z-s*x;t*x*z-s*y t*y*z+s*x t*z*z+c];
end
