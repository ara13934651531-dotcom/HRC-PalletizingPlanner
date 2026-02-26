function testElfin5_Palletizing()
%% testElfin5_Palletizing - Elfin5 码垛工作站仿真
%  基于 testS50_Palletizing_v11 改编, 适配 Elfin5 (5kg) 小型码垛场景
%  使用 MATLAB Robotics Toolbox IK 求解 (100%成功率)
%
%  功能:
%    1. hgtransform 胶囊体机器人 (碰撞模型外观)
%    2. 小型码垛场景: 电箱 + 工作台 + 篮筐 + 传送带 + 箱子
%    3. Robotics Toolbox IK + 6箱取放动画 (3x2层)
%    4. TCP轨迹显示 + 实时信息面板
%    5. Headless兼容 (自动保存PNG/GIF)
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

close all; clc;
addpath('collisionVisual'); addpath(genpath('collisionVisual'));

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                    用户可配置参数区                                  ║
%% ╚══════════════════════════════════════════════════════════════════════╝
% Elfin5 臂展~800mm, 场景紧凑设计
% 基座高度0.28m, 肩关节高0.50m, 水平臂展a2+d4=0.80m

% 电箱 (机器人基座平台)
cfg_cab.widthX = 0.24; cfg_cab.depthY = 0.24; cfg_cab.heightZ = 0.28;
cfg_cab.color = [0.95, 0.95, 0.93];

% 篮筐 (码垛目标区) — 架高, 放在机器人侧前方
cfg_frame.widthX = 0.40; cfg_frame.depthY = 0.32; cfg_frame.height = 0.55;
cfg_frame.tubeR = 0.012; cfg_frame.color = [0.25, 0.55, 0.85];
cfg_frame.tableH = 0.18;  % 篮筐底部抬高 (工作台)

% 托盘 — 放在篮筐底部(在工作台上)
cfg_pallet.widthX = 0.34; cfg_pallet.depthY = 0.28; cfg_pallet.heightZ = 0.04;
cfg_pallet.color = [0.20, 0.45, 0.80];

% 传送带 — 放在机器人右侧(X+方向), 高度与码垛区接近
cfg_conv.lengthY = 0.50; cfg_conv.widthX = 0.20; cfg_conv.heightZ = 0.18;
cfg_conv.beltH = 0.012; cfg_conv.rollerR = 0.010; cfg_conv.nRollers = 6;
cfg_conv.color = [0.30, 0.30, 0.32];

% 箱子 — 小型 (Elfin5 负载5kg)
cfg_box.lx = 0.10; cfg_box.wy = 0.08; cfg_box.hz = 0.06;
cfg_box.color = [0.65, 0.45, 0.25];
cfg_nBoxes = 6;

cfg_frameGap = 0.12;    % 电箱到篮筐间距
cfg_convGap  = 0.10;    % 电箱到传送带间距
cfg_convOffY = -0.04;
cfg_convBoxYStart = 0.12;
cfg_convBoxYStep  = 0.10;

% 动画参数
INTERP_STEPS = 4;
PAUSE_INTERP = 0.001;

% 渲染参数
CYL_N = 6;
CAPSULE_N = 8;
CAPSULE_ALPHA = 0.35;
VIEW_AZ = 135; VIEW_EL = 25;

CAPSULE_COLORS = {[0.75 0.75 0.80],[0.50 0.55 0.85],[0.85 0.50 0.45],...
                  [0.50 0.80 0.55],[0.80 0.75 0.50]};

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                      系统初始化                                     ║
%% ╚══════════════════════════════════════════════════════════════════════╝
CJK_FONT = 'Noto Sans CJK SC';
allFonts = listfonts();
if ~any(strcmp(allFonts, CJK_FONT))
    candidates = {'Noto Serif CJK SC','SimHei','WenQuanYi Micro Hei','Arial Unicode MS'};
    CJK_FONT = '';
    for ci = 1:length(candidates)
        if any(strcmp(allFonts, candidates{ci})), CJK_FONT = candidates{ci}; break; end
    end
    if isempty(CJK_FONT), CJK_FONT = get(0, 'DefaultAxesFontName'); end
end
MONO_FONT = CJK_FONT;
set(0, 'DefaultAxesFontName', CJK_FONT);
set(0, 'DefaultTextFontName', CJK_FONT);

isHeadless = ~usejava('desktop');
outputDir = './pic/elfin5_palletizing';
if isHeadless, set(0, 'DefaultFigureVisible', 'off'); end
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

cab = cfg_cab; frame = cfg_frame; pallet = cfg_pallet;
conv = cfg_conv; box = cfg_box; nBoxes = cfg_nBoxes;
baseX = 0; baseY = 0; baseZ = cab.heightZ;

frame.cx = 0.0;
frame.cy = cab.depthY/2 + cfg_frameGap + frame.depthY/2;
conv.cx = cab.widthX/2 + cfg_convGap + conv.widthX/2;
conv.cy = cfg_convOffY;

convSurfZ   = conv.heightZ + conv.rollerR + conv.beltH;
convBoxTopZ = convSurfZ + box.hz;

% 码放区在工作台上方
frameTableTopZ = frame.tableH;     % 工作台顶面
palletTopZ = frameTableTopZ + pallet.heightZ;  % 托盘顶面

Tb = eye(4); Tb(1,4) = baseX; Tb(2,4) = baseY; Tb(3,4) = baseZ;

% Elfin5 DH参数 (m)
DH_d1 = 0.220;
DH_a2 = 0.380;
DH_d4 = 0.420;
DH_d6 = 0.1565;

% 碰撞模型几何定义 (m, 与 elfin5_collision.json 一致)
capsuleDefs = {
    'base',     [0 0 0.05],      [0 0 0.2],       0.10, 1;
    'lowerArm', [-0.01 0 -0.125],[-0.38 0 -0.125],0.06, 3;
    'elbow',    [0 0 -0.03],     [0 0 0.15],       0.08, 4;
    'upperArm', [0 0 -0.09],     [0 0.2 -0.09],   0.045, 5;
};
sphereDef_offset = [0 -0.025 0.05]; sphereDef_radius = 0.11; sphereDef_linkIdx = 6;

fprintf('\n');
fprintf([char(9556) repmat(char(9552),1,72) char(9559) '\n']);
fprintf([char(9553) '  Elfin5 Palletizing -- Robotics Toolbox IK + Capsule Animation       ' char(9553) '\n']);
fprintf([char(9562) repmat(char(9552),1,72) char(9565) '\n\n']);

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║             加载URDF: importrobot + IK求解器                        ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('[1] Loading URDF model...\n');
urdfPath = './model/urdf/elfin5.urdf';
robot = importrobot(urdfPath);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% IK求解器
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [0.5 0.5 0.5 1.0 1.0 1.0]; % [orientation(3) position(3)]
endEffector = robot.BodyNames{end};
fprintf('  Robot: %s, end-effector: %s\n', 'elfin5', endEffector);

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║      箱子排列: 传送带一列 + 篮筐 3x2层                             ║
%% ╚══════════════════════════════════════════════════════════════════════╝
% 传送带上箱子 Y 位置
convBoxY = zeros(1, nBoxes);
for bi = 1:nBoxes
    convBoxY(bi) = conv.cy + cfg_convBoxYStart - (bi-1)*cfg_convBoxYStep;
end
convBoxX = conv.cx;

% 篮筐内码放位置 (工作台上方)
palletYStart = frame.cy - pallet.depthY/2 + 0.01;
boxSpacingY  = box.wy + 0.01;

placePos = zeros(nBoxes, 3);
for bi = 1:nBoxes
    layer = ceil(bi / 3);
    col   = mod(bi-1, 3) + 1;
    placePos(bi,:) = [frame.cx, palletYStart + (col-0.5)*boxSpacingY, ...
                      palletTopZ + (layer-0.5)*box.hz];
end

fprintf('\nLayout: 3 boxes/row x 2 layers = 6 boxes\n');
fprintf('Base height: %.3fm, Conveyor surface: %.3fm, Pallet top: %.3fm\n', ...
    baseZ, convSurfZ, palletTopZ);
for bi = 1:nBoxes
    fprintf('  Box%d: conveyor=[%.3f,%.3f,%.3f] pallet=[%.3f,%.3f,%.3f]\n', ...
        bi, convBoxX, convBoxY(bi), convBoxTopZ, placePos(bi,:));
end
fprintf('\n');

%% ---- Elfin5 独立FK函数 (用于碰撞模型渲染) ----
    function T_all = FK_local(qin)
        dhTab = [qin(1),        DH_d1, 0,      pi/2;
                 qin(2)+pi/2,   0,     DH_a2,  pi;
                 qin(3)+pi/2,   0,     0,      pi/2;
                 qin(4),        DH_d4, 0,      -pi/2;
                 qin(5),        0,     0,      pi/2;
                 qin(6),        DH_d6, 0,      0];
        Tl = cell(6,1);
        for ii = 1:6
            ct = cos(dhTab(ii,1)); st = sin(dhTab(ii,1));
            ca = cos(dhTab(ii,4)); sa = sin(dhTab(ii,4));
            Tl{ii} = [ct -st*ca  st*sa  dhTab(ii,3)*ct;
                      st  ct*ca -ct*sa  dhTab(ii,3)*st;
                      0   sa     ca     dhTab(ii,2);
                      0   0      0      1];
        end
        T00 = eye(4); T01 = Tl{1}; T02 = T01*Tl{2}; T03 = T02*Tl{3};
        T04 = T03*Tl{4}; T05 = T04*Tl{5}; T06 = T05*Tl{6};
        T_all = {T00, T01, T02, T03, T04, T05, T06};
    end

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║           MATLAB Robotics Toolbox IK 求解                           ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('[2] IK solving with Robotics Toolbox...\n');

% 辅助: 构建朝下末端姿态 (Z轴对齐世界-Z)
    function tform = makeDownTform(x, y, z)
        % 工具末端朝下: R_z轴 = [0;0;-1]
        tform = [1  0  0  x;
                 0 -1  0  y;
                 0  0 -1  z;
                 0  0  0  1];
    end

q_home_guess = [0 0 0 0 0 0]; % 初始猜测

% Pick IK
pickIK_q = zeros(nBoxes, 6);
fprintf('  --- Pick positions ---\n');
for bi = 1:nBoxes
    tgt = makeDownTform(convBoxX, convBoxY(bi), convBoxTopZ);
    [qSol, solInfo] = ik(endEffector, tgt, ikWeights, q_home_guess);
    pickIK_q(bi,:) = qSol;
    T_fk = getTransform(robot, qSol, endEffector);
    posErr = norm(T_fk(1:3,4) - tgt(1:3,4)) * 1000;
    tag = ''; if posErr > 5, tag = ' *** WARN ***'; end
    fprintf('  Pick%d: err=%.2fmm status=%d%s\n', bi, posErr, solInfo.ExitFlag, tag);
    q_home_guess = qSol;  % 使用上一个解作为下一个初始猜测
end

% Place IK
placeIK_q = zeros(nBoxes, 6);
fprintf('  --- Place positions ---\n');
q_home_guess = [0 pi/4 0 0 0 0]; % 朝篮筐方向的初始猜测
for bi = 1:nBoxes
    tgt = makeDownTform(placePos(bi,1), placePos(bi,2), placePos(bi,3));
    [qSol, solInfo] = ik(endEffector, tgt, ikWeights, q_home_guess);
    placeIK_q(bi,:) = qSol;
    T_fk = getTransform(robot, qSol, endEffector);
    posErr = norm(T_fk(1:3,4) - tgt(1:3,4)) * 1000;
    tag = ''; if posErr > 5, tag = ' *** WARN ***'; end
    fprintf('  Place%d: err=%.2fmm status=%d%s\n', bi, posErr, solInfo.ExitFlag, tag);
    q_home_guess = qSol;
end
fprintf('\n');

%% ---- 姿态序列 ----
% Home 姿态 (臂收起)
q_home = [0, deg2rad(-30), deg2rad(40), 0, deg2rad(-45), 0];
% 安全过渡姿态 (高处)
q_safe = [deg2rad(-45), deg2rad(-40), deg2rad(30), deg2rad(-30), deg2rad(-60), 0];

    function q_hov = makeHover(q_target)
        % 在目标姿态基础上微调 q2/q3 实现悬停 (稍高)
        q_hov = q_target;
        q_hov(2) = q_hov(2) + deg2rad(6);
        q_hov(3) = q_hov(3) - deg2rad(5);
    end

boxActions = cell(1, nBoxes);
for bi = 1:nBoxes
    q_pick   = pickIK_q(bi,:);
    q_place  = placeIK_q(bi,:);
    q_hov_pk = makeHover(q_pick);
    q_hov_pl = makeHover(q_place);

    boxActions{bi} = {
        q_home,     'Home',           false;
        q_hov_pk,   'Hover Pick',     false;
        q_pick,     'Pick',           false;
        q_hov_pk,   'Lift',           true;
        q_safe,     'Safe Transit',   true;
        q_hov_pl,   'Hover Place',    true;
        q_place,    'Place',          true;
        q_hov_pl,   'Retract Up',     false;
        q_safe,     'Safe Retract',   false;
        q_home,     'Return Home',    false;
    };
end

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  渲染: 静态场景 + hgtransform 胶囊体机器人                          ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('[3] Rendering scene + robot model...\n');
totalStartTime = tic;

fig = figure('Position', [50 50 1700 950], 'Color', 'w', 'Renderer', 'opengl', ...
    'Name', 'Elfin5 Palletizing');

ax3d = axes('Parent', fig, 'Position', [0.02 0.05 0.64 0.88]);
hold(ax3d, 'on');

% --- 静态场景 ---
drawGround_v11(ax3d, -0.4, 0.6, -0.5, 0.8);
drawCabinet_v11(ax3d, cab, baseX, baseY);

% 工作台 (篮筐底座)
drawBox3D_v11(ax3d, frame.cx-frame.widthX/2-0.02, frame.cy-frame.depthY/2-0.02, ...
    0, frame.widthX+0.04, frame.depthY+0.04, frame.tableH, [0.55 0.55 0.50]);

drawFrame_v11(ax3d, frame, CYL_N);
drawPallet_v11(ax3d, pallet, frame, frameTableTopZ, CJK_FONT);
drawConveyor_v11(ax3d, conv, CYL_N);

xlabel(ax3d,'X (m)','FontSize',14,'FontWeight','bold','FontName',CJK_FONT);
ylabel(ax3d,'Y (m)','FontSize',14,'FontWeight','bold','FontName',CJK_FONT);
zlabel(ax3d,'Z (m)','FontSize',14,'FontWeight','bold','FontName',CJK_FONT);
set(ax3d,'FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
axis(ax3d,'equal'); grid(ax3d,'on');
xlim(ax3d,[-0.4 0.6]); ylim(ax3d,[-0.5 0.8]); zlim(ax3d,[0 0.9]);
view(ax3d,VIEW_AZ,VIEW_EL);
camlight('headlight'); lighting(ax3d,'gouraud');

hTitle = title(ax3d, 'Elfin5 Palletizing | Initializing...', ...
    'FontSize', 18, 'FontWeight', 'bold', 'FontName', CJK_FONT, 'Color', [0.1 0.1 0.3]);

% 信息面板
ax_info = axes('Parent', fig, 'Position', [0.68 0.05 0.30 0.88]);
axis(ax_info, 'off'); xlim(ax_info,[0 1]); ylim(ax_info,[0 1]); hold(ax_info,'on');

% 传送带箱子
hConvBoxes  = gobjects(nBoxes, 1);
hConvLabels = gobjects(nBoxes, 1);
bz_center = convSurfZ + box.hz/2;
for ci = 1:nBoxes
    hConvBoxes(ci) = drawBox_v11(ax3d, [convBoxX, convBoxY(ci), bz_center], box);
    hConvLabels(ci) = text(ax3d, convBoxX, convBoxY(ci), convSurfZ+box.hz+0.03, ...
        sprintf('#%d',ci), 'FontSize', 12, 'FontWeight', 'bold', 'FontName', CJK_FONT, ...
        'HorizontalAlignment', 'center', 'Color', [.8 .3 0]);
end
hPlacedBoxes  = gobjects(nBoxes, 1);
hPlacedLabels = gobjects(nBoxes, 1);

%% --- 创建 hgtransform 胶囊体机器人 ---
nCaps = size(capsuleDefs, 1);
hCapsuleHT = gobjects(nCaps, 1);
capsuleLocalT = cell(nCaps, 1);
capsuleLinkIdx = zeros(nCaps, 1);

for ci = 1:nCaps
    p1 = capsuleDefs{ci,2};
    p2 = capsuleDefs{ci,3};
    radius = capsuleDefs{ci,4};
    linkId = capsuleDefs{ci,5};
    capsuleLinkIdx(ci) = linkId;

    ht = hgtransform('Parent', ax3d);
    hCapsuleHT(ci) = ht;

    L = norm(p2 - p1);
    [Xc,Yc,Zc] = capsuleMesh_v11(radius, L, CAPSULE_N);
    col = CAPSULE_COLORS{min(ci, length(CAPSULE_COLORS))};
    surf(Xc, Yc, Zc, 'Parent', ht, 'FaceColor', col, 'EdgeColor', 'none', ...
        'FaceAlpha', CAPSULE_ALPHA, 'FaceLighting', 'gouraud');

    mid = (p1 + p2) / 2;
    if L > 1e-6
        d_vec = (p2 - p1) / L;
        R = rotZtoDir(d_vec);
    else
        R = eye(3);
    end
    Tlocal = eye(4);
    Tlocal(1:3,1:3) = R;
    Tlocal(1:3,4) = mid(:);
    capsuleLocalT{ci} = Tlocal;
end

% 腕关节球体
htWrist = hgtransform('Parent', ax3d);
[Xs,Ys,Zs] = sphere(CAPSULE_N);
Xs = Xs*sphereDef_radius; Ys = Ys*sphereDef_radius; Zs = Zs*sphereDef_radius;
surf(Xs, Ys, Zs, 'Parent', htWrist, 'FaceColor', [0.80 0.75 0.50], ...
    'EdgeColor', 'none', 'FaceAlpha', CAPSULE_ALPHA, 'FaceLighting', 'gouraud');
TWristLocal = eye(4); TWristLocal(1:3,4) = sphereDef_offset(:);

% TCP和关节标记
hTcpMarker = plot3(ax3d, 0, 0, baseZ, 'rp', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'r', 'LineWidth', 1.5);
hJointDots = scatter3(ax3d, zeros(8,1), zeros(8,1), zeros(8,1), ...
    50, 'k', 'filled', 'MarkerEdgeColor', [0.3 0.3 0.3]);

hTrailLine = gobjects(0);
hCarryBox  = gobjects(0);
hTargetMark = gobjects(0);

drawnow;
fprintf('  Scene + robot rendered in %.2fs\n', toc(totalStartTime));

%% ---- updateRobot ----
    function tcp = updateRobot(q)
        T_all = FK_local(q);
        Tw = cell(7,1);
        for ii = 1:7, Tw{ii} = Tb * T_all{ii}; end

        for ci2 = 1:nCaps
            li = capsuleLinkIdx(ci2);
            set(hCapsuleHT(ci2), 'Matrix', Tw{li} * capsuleLocalT{ci2});
        end
        set(htWrist, 'Matrix', Tw{sphereDef_linkIdx} * TWristLocal);

        tcp = Tw{7}(1:3,4)';
        set(hTcpMarker, 'XData', tcp(1), 'YData', tcp(2), 'ZData', tcp(3));

        jp = zeros(8,3); jp(1,:) = [baseX, baseY, baseZ];
        for ii = 1:7, jp(ii+1,:) = Tw{ii}(1:3,4)'; end
        set(hJointDots, 'XData', jp(:,1), 'YData', jp(:,2), 'ZData', jp(:,3));
    end

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                     动画主循环                                      ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('[4] Starting animation (%d boxes, %d interp steps)...\n\n', nBoxes, INTERP_STEPS);
allGifFrames = {};
animStartTime = tic;

for bi = 1:nBoxes
    boxStartClock = clock;
    tcpTrail = [];
    nActions = size(boxActions{bi}, 1) - 1;

    if ~isempty(hTrailLine) && any(isvalid(hTrailLine))
        delete(hTrailLine(isvalid(hTrailLine)));
    end
    hTrailLine = gobjects(0);

    fprintf('=== Box %d/%d ===\n', bi, nBoxes);

    for ai = 1:nActions
        q_start = boxActions{bi}{ai, 1};
        q_end   = boxActions{bi}{ai+1, 1};
        carrying_start = boxActions{bi}{ai, 3};
        carrying_end   = boxActions{bi}{ai+1, 3};
        actionName = boxActions{bi}{ai+1, 2};
        carrying = carrying_start || carrying_end;
        if strcmp(actionName, 'Pick'), carrying = false; end

        for si = 0:INTERP_STEPS
            t_frac = si / INTERP_STEPS;
            q_curr = (1-t_frac) * q_start + t_frac * q_end;

            tcp = updateRobot(q_curr);
            tcpTrail = [tcpTrail; tcp];

            % 携带箱子
            if ~isempty(hCarryBox) && any(isvalid(hCarryBox))
                delete(hCarryBox(isvalid(hCarryBox)));
            end
            hCarryBox = gobjects(0);
            if carrying
                hCarryBox = drawBox_v11(ax3d, [tcp(1), tcp(2), tcp(3)-0.005], box);
            end

            % 轨迹线
            if ~isempty(hTrailLine) && any(isvalid(hTrailLine))
                delete(hTrailLine(isvalid(hTrailLine)));
            end
            hTrailLine = gobjects(0);
            if size(tcpTrail,1) > 1
                hTrailLine = plot3(ax3d, tcpTrail(:,1), tcpTrail(:,2), tcpTrail(:,3), ...
                    '-', 'Color', [1 0.3 0 0.85], 'LineWidth', 2.5);
            end

            % 目标标记
            if ~isempty(hTargetMark) && any(isvalid(hTargetMark))
                delete(hTargetMark(isvalid(hTargetMark)));
            end
            hTargetMark = gobjects(0);
            if any(strcmp(actionName, {'Pick','Hover Pick','Lift'}))
                hTargetMark = plot3(ax3d, convBoxX, convBoxY(bi), convBoxTopZ, 'gv', ...
                    'MarkerSize', 14, 'LineWidth', 2.5, 'MarkerFaceColor', [0 0.8 0]);
            elseif any(strcmp(actionName, {'Place','Hover Place','Retract Up'}))
                hTargetMark = plot3(ax3d, placePos(bi,1), placePos(bi,2), placePos(bi,3), 'rv', ...
                    'MarkerSize', 14, 'LineWidth', 2.5, 'MarkerFaceColor', [0.8 0 0]);
            end

            % 传送带箱子可见性
            for ci2 = 1:nBoxes
                if ci2 < bi || (ci2 == bi && carrying)
                    set(hConvBoxes(ci2),'Visible','off'); set(hConvLabels(ci2),'Visible','off');
                else
                    set(hConvBoxes(ci2),'Visible','on'); set(hConvLabels(ci2),'Visible','on');
                end
            end

            set(hTitle, 'String', sprintf('Elfin5 Palletizing | Box %d/%d: %s [%d/%d]', ...
                bi, nBoxes, actionName, ai, nActions));

            cla(ax_info);
            boxElapsed = etime(clock, boxStartClock);
            drawInfoPanel_v11(ax_info, bi, nBoxes, actionName, tcp, q_curr, ...
                boxElapsed, carrying, CJK_FONT, MONO_FONT, toc(totalStartTime), ai, nActions);

            drawnow limitrate;
            pause(PAUSE_INTERP);
        end

        if strcmp(actionName, 'Pick')
            set(hConvBoxes(bi),'Visible','off'); set(hConvLabels(bi),'Visible','off');
        end
        if strcmp(actionName, 'Place')
            hPlacedBoxes(bi) = drawBox_v11(ax3d, placePos(bi,:), box);
            hPlacedLabels(bi) = text(ax3d, placePos(bi,1), placePos(bi,2), ...
                placePos(bi,3)+box.hz/2+0.03, sprintf('#%d',bi), ...
                'FontSize', 11, 'FontWeight', 'bold', 'FontName', CJK_FONT, ...
                'HorizontalAlignment', 'center', 'Color', [.0 .4 .7]);
        end

        fprintf('  [%d/%d] %-14s TCP=[%+.3f,%+.3f,%+.3f] t=%.1fs\n', ...
            ai, nActions, actionName, tcp, etime(clock, boxStartClock));

        if isHeadless && any(strcmp(actionName, {'Pick','Place','Return Home'}))
            allGifFrames{end+1} = getframe(fig);
        end
    end

    boxTime = etime(clock, boxStartClock);
    fprintf('  Box %d complete in %.2fs\n\n', bi, boxTime);
end

totalTime = toc(totalStartTime);
fprintf('================================\n');
fprintf('Total animation: %.2fs for %d boxes\n', totalTime, nBoxes);
fprintf('================================\n');

%% ---- 保存 ----
if isHeadless
    fn = sprintf('%s/palletizing_final.png', outputDir);
    print(fig, fn, '-dpng', '-r150');
    fprintf('Saved: %s\n', fn);

    if ~isempty(allGifFrames)
        gifFile = sprintf('%s/elfin5_palletizing.gif', outputDir);
        for gi = 1:length(allGifFrames)
            [A, map] = rgb2ind(allGifFrames{gi}.cdata, 128);
            if gi == 1
                imwrite(A, map, gifFile, 'gif', 'LoopCount', 0, 'DelayTime', 0.8);
            else
                imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.8);
            end
        end
        fprintf('GIF: %s (%d frames)\n', gifFile, length(allGifFrames));
    end
    close(fig);
end
fprintf('\nElfin5 Palletizing complete!\n');


%% ========================================================================
%%                       辅助函数
%% ========================================================================

function [X,Y,Z] = capsuleMesh_v11(radius, L, nSeg)
    nHemi = max(3, floor(nSeg/2));
    tBot = linspace(-pi/2, 0, nHemi+1);
    rBot = radius * cos(tBot); zBot = radius * sin(tBot) - L/2;
    rCyl = [radius; radius]; zCyl = [-L/2; L/2];
    tTop = linspace(0, pi/2, nHemi+1);
    rTop = radius * cos(tTop); zTop = radius * sin(tTop) + L/2;
    rAll = [rBot(:); rCyl(:); rTop(:)];
    zAll = [zBot(:); zCyl(:); zTop(:)];
    theta = linspace(0, 2*pi, nSeg+1);
    X = rAll * cos(theta);
    Y = rAll * sin(theta);
    Z = repmat(zAll, 1, nSeg+1);
end

function R = rotZtoDir(d)
    d = d(:)/norm(d); z = [0;0;1];
    v = cross(z, d); s = norm(v); c = dot(z, d);
    if s < 1e-10
        if c > 0, R = eye(3); else, R = diag([-1,-1,1]); end; return;
    end
    vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    R = eye(3) + vx + vx*vx*(1-c)/(s*s);
end

function h = drawBox_v11(ax, pos, bx)
    x = pos(1)-bx.lx/2; y = pos(2)-bx.wy/2; z = pos(3)-bx.hz/2;
    v = [x y z;x+bx.lx y z;x+bx.lx y+bx.wy z;x y+bx.wy z;
         x y z+bx.hz;x+bx.lx y z+bx.hz;x+bx.lx y+bx.wy z+bx.hz;x y+bx.wy z+bx.hz];
    h = patch(ax,'Vertices',v, ...
        'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
        'FaceColor',bx.color,'EdgeColor',[.35 .25 .1],'FaceAlpha',.95,'LineWidth',1.2);
end

function drawInfoPanel_v11(ax, bi, nBoxes, actionName, tcp, q, ...
    actionTime, carrying, cjkFont, monoFont, totalTime, stepIdx, totalSteps)
    rectangle(ax,'Position',[0 0 1 1],'FaceColor',[0.97 0.97 0.99],...
        'EdgeColor',[0.5 0.5 0.7],'LineWidth',2,'Curvature',0.02);
    yP = 0.96;
    text(ax,0.5,yP,'PALLETIZING DASHBOARD','FontSize',16,'FontWeight','bold',...
        'HorizontalAlignment','center','Color',[0.1 0.1 0.3],'FontName',cjkFont);
    yP = yP - 0.04;
    text(ax,0.5,yP,'Elfin5 (5kg)','FontSize',13,'FontWeight','bold',...
        'HorizontalAlignment','center','Color',[0.3 0.3 0.5],'FontName',cjkFont);
    yP = yP - 0.06;
    rectangle(ax,'Position',[0.05 yP-0.03 0.90 0.045],'FaceColor',[0.15 0.35 0.65],...
        'EdgeColor','none','Curvature',0.3);
    progW = max(0.01, 0.86*(bi/nBoxes));
    rectangle(ax,'Position',[0.07 yP-0.025 progW 0.035],'FaceColor',[0.3 0.75 0.45],...
        'EdgeColor','none','Curvature',0.3);
    text(ax,0.5,yP-0.008,sprintf('Box %d/%d | Step %d/%d',bi,nBoxes,stepIdx,totalSteps),...
        'FontSize',13,'FontWeight','bold','HorizontalAlignment','center','Color','w','FontName',cjkFont);
    yP = yP - 0.07;
    if carrying, actC=[0.8 0.4 0];actBg=[1 0.95 0.88];cTxt=' [CARRYING]';
    else, actC=[0.1 0.5 0.2];actBg=[0.9 1 0.92];cTxt=''; end
    rectangle(ax,'Position',[0.05 yP-0.02 0.90 0.04],'FaceColor',actBg,...
        'EdgeColor',actC,'LineWidth',2,'Curvature',0.2);
    text(ax,0.5,yP,sprintf('%s%s',actionName,cTxt),'FontSize',14,'FontWeight','bold',...
        'HorizontalAlignment','center','Color',actC,'FontName',cjkFont);
    yP = yP - 0.065;
    text(ax,0.05,yP,'TCP Position (mm)','FontSize',14,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP = yP - 0.038;
    labels = {'X','Y','Z'}; colors = {[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8]};
    for ci2 = 1:3
        text(ax,0.08,yP,sprintf('%s: %+8.1f',labels{ci2},tcp(ci2)*1000),...
            'FontSize',14,'FontWeight','bold','Color',colors{ci2},'FontName',monoFont);
        yP = yP - 0.034;
    end
    yP = yP - 0.012;
    text(ax,0.05,yP,'Joint Angles (deg)','FontSize',14,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP = yP - 0.034;
    q_deg = rad2deg(q);
    for ji = 1:6
        barMax = 0.50; barFrac = abs(q_deg(ji))/180; barW = max(0.01, barFrac*barMax);
        if q_deg(ji) >= 0, bC = [0.3 0.6 0.9]; else, bC = [0.9 0.5 0.2]; end
        rectangle(ax,'Position',[0.22 yP-0.010 barMax 0.018],'FaceColor',[0.92 0.92 0.92],'EdgeColor','none');
        rectangle(ax,'Position',[0.22 yP-0.010 barW 0.018],'FaceColor',bC,'EdgeColor','none','Curvature',0.3);
        text(ax,0.06,yP-0.001,sprintf('J%d',ji),'FontSize',11,'FontWeight','bold','Color',[0.3 0.3 0.3],'FontName',cjkFont);
        text(ax,0.76,yP-0.001,sprintf('%+7.1f',q_deg(ji)),'FontSize',12,'FontWeight','bold','Color',bC,'FontName',monoFont);
        yP = yP - 0.028;
    end
    yP = yP - 0.015;
    text(ax,0.05,yP,'Timing','FontSize',14,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP = yP - 0.038;
    text(ax,0.08,yP,sprintf('Box time:  %.2f s',actionTime),'FontSize',13,'FontWeight','bold','Color',[0.2 0.2 0.2],'FontName',monoFont);
    yP = yP - 0.030;
    text(ax,0.08,yP,sprintf('Total:     %.2f s',totalTime),'FontSize',13,'FontWeight','bold','Color',[0.2 0.2 0.2],'FontName',monoFont);
    yP = yP - 0.030;
    if bi < nBoxes
        est = totalTime/bi*(nBoxes-bi);
        text(ax,0.08,yP,sprintf('Remaining: ~%.1f s',est),'FontSize',13,'FontWeight','bold','Color',[0.5 0.3 0],'FontName',monoFont);
    else
        text(ax,0.08,yP,'COMPLETING...','FontSize',13,'FontWeight','bold','Color',[0 0.6 0.2],'FontName',monoFont);
    end
end

%% ---- 静态场景绘制 ----
function drawGround_v11(ax,x0,x1,y0,y1)
    patch(ax,'Vertices',[x0 y0 0;x1 y0 0;x1 y1 0;x0 y1 0],'Faces',[1 2 3 4],...
          'FaceColor',[.92 .92 .90],'EdgeColor','none','FaceAlpha',0.4);
end
function drawCabinet_v11(ax,c,bx,by)
    x=bx-c.widthX/2;y=by-c.depthY/2;w=c.widthX;d=c.depthY;h=c.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',c.color,'EdgeColor',[.5 .5 .5],'FaceAlpha',.95,'LineWidth',1);
end
function drawFrame_v11(ax,f,cylN)
    r=f.tubeR;wx=f.widthX;dy=f.depthY;h=f.height;cx=f.cx;cy=f.cy;
    zOff = f.tableH;  % 篮筐底部起始高度
    c=[cx-wx/2 cy-dy/2;cx+wx/2 cy-dy/2;cx+wx/2 cy+dy/2;cx-wx/2 cy+dy/2];
    for i=1:4, drawTube_v11(ax,c(i,1),c(i,2),zOff,c(i,1),c(i,2),zOff+h,r,f.color,cylN); end
    edges={[1,2],[2,3],[3,4],[4,1]};
    for hz=[zOff+0.03 zOff+h/3 zOff+2*h/3 zOff+h-0.03]
        for ei=2:4
            i1=edges{ei}(1);i2=edges{ei}(2);
            drawTube_v11(ax,c(i1,1),c(i1,2),hz,c(i2,1),c(i2,2),hz,r*.8,f.color,cylN);
        end
    end
end
function drawPallet_v11(ax,pal,frm,tableH,fontName)
    x=frm.cx-pal.widthX/2;y=frm.cy-pal.depthY/2;
    w=pal.widthX;d=pal.depthY;h=pal.heightZ;
    zBase = tableH;
    v=[x y zBase;x+w y zBase;x+w y+d zBase;x y+d zBase;
       x y zBase+h;x+w y zBase+h;x+w y+d zBase+h;x y+d zBase+h];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',pal.color,'EdgeColor',[.15 .30 .60],'FaceAlpha',.90,'LineWidth',1.2);
    text(ax,frm.cx,frm.cy,zBase+h+0.02,sprintf('Pallet'),...
         'FontSize',11,'HorizontalAlignment','center','Color',[.05 .15 .45],...
         'FontWeight','bold','FontName',fontName);
end
function drawConveyor_v11(ax,cv,cylN)
    cx=cv.cx;cy=cv.cy;ly=cv.lengthY;wx=cv.widthX;hz=cv.heightZ;
    x0=cx-wx/2;y0=cy-ly/2;
    drawBox3D_v11(ax,x0-.01,y0,hz-.03,.01,ly,.03,[.4 .4 .42]);
    drawBox3D_v11(ax,x0+wx,y0,hz-.03,.01,ly,.03,[.4 .4 .42]);
    lw=.015;yL=[y0+.08 cy y0+ly-.08];
    for yi=1:3
        for s=[-1 1]
            lx=cx+s*(wx/2-.02);
            drawBox3D_v11(ax,lx-lw/2,yL(yi)-lw/2,0,lw,lw,hz-.003,[.25 .25 .25]);
        end
    end
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        [X,Y,Z]=cylinder(cv.rollerR,cylN); Z=Z*wx*.9-wx*.9/2;
        surf(ax,Z+cx,zeros(size(X))+ry,X+hz+cv.rollerR,'FaceColor',[.55 .55 .55],'EdgeColor','none','FaceAlpha',.6);
    end
    bz=hz+cv.rollerR;
    bV=[x0+.01 y0+.02 bz;x0+wx-.01 y0+.02 bz;x0+wx-.01 y0+ly-.02 bz;x0+.01 y0+ly-.02 bz;
        x0+.01 y0+.02 bz+cv.beltH;x0+wx-.01 y0+.02 bz+cv.beltH;
        x0+wx-.01 y0+ly-.02 bz+cv.beltH;x0+.01 y0+ly-.02 bz+cv.beltH];
    patch(ax,'Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[.15 .15 .15],'FaceAlpha',.92);
end
function drawBox3D_v11(ax,x,y,z,dx,dy,dz,col)
    v=[x y z;x+dx y z;x+dx y+dy z;x y+dy z;x y z+dz;x+dx y z+dz;x+dx y+dy z+dz;x y+dy z+dz];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',col,'EdgeColor','none','FaceAlpha',.95);
end
function drawTube_v11(ax,x1,y1,z1,x2,y2,z2,radius,color,cylN)
    [X,Y,Z]=cylinder(radius,cylN);
    v=[x2-x1;y2-y1;z2-z1];l=norm(v);
    if l<.001,return;end
    Z=Z*l;dd=[0;0;1];td=v/l;cp=cross(dd,td);
    if norm(cp)>1e-6
        RR=axang2r_v11([cp'/norm(cp),acos(max(-1,min(1,dot(dd,td))))]);
    else
        RR=eye(3);if dot(dd,td)<0,RR(3,3)=-1;RR(1,1)=-1;end
    end
    for i=1:numel(X)
        pt=RR*[X(i);Y(i);Z(i)];X(i)=pt(1)+x1;Y(i)=pt(2)+y1;Z(i)=pt(3)+z1;
    end
    surf(ax,X,Y,Z,'FaceColor',color,'EdgeColor','none','FaceAlpha',.85);
end
function R=axang2r_v11(ax)
    a=ax(1:3);g=ax(4);c=cos(g);s=sin(g);t=1-c;x=a(1);y=a(2);z=a(3);
    R=[t*x*x+c t*x*y-s*z t*x*z+s*y;t*x*y+s*z t*y*y+c t*y*z-s*x;t*x*z-s*y t*y*z+s*x t*z*z+c];
end

end  % testElfin5_Palletizing main function
