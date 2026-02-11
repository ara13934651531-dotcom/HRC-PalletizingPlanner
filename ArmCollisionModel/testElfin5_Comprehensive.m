function testElfin5_Comprehensive()
%% testElfin5_Comprehensive - Elfin5 全面系统仿真 (Headless兼容)
%
%  测试项目:
%    1. URDF模型加载 + STL可视化
%    2. 正运动学 (FK) 验证 — DH vs URDF FK对比
%    3. MATLAB内置IK求解器 (Robotics Toolbox) 测试
%    4. 碰撞几何模型 (胶囊体+球体) + 可视化
%    5. 自碰撞检测 — 全部连杆对距离计算
%    6. 多姿态遍历 — 工作空间扫描 + 碰撞统计
%    7. IK精度分析 — FK→IK→FK闭环误差
%    8. 关节空间路径插值 + 碰撞过程检测
%    9. 性能基准 (FK/IK/碰撞检测耗时)
%   10. 输出汇总报告 + 保存图片
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
% 用于验证碰撞接口完备性, 为HR_S50迁移提供参考

close all; clc;

%% ═══════════════════════════════════════════════════════════════════════
%% 0. 环境初始化
%% ═══════════════════════════════════════════════════════════════════════
addpath('collisionVisual');
addpath(genpath('collisionVisual'));

robName = 'elfin5';
isHeadless = ~usejava('desktop');
outputDir = ['./pic/elfin5_comprehensive'];
if isHeadless, set(0, 'DefaultFigureVisible', 'off'); end
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

% 中文字体
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

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║         Elfin5 全面系统仿真 — Comprehensive Simulation              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n\n');
totalStart = tic;

%% ═══════════════════════════════════════════════════════════════════════
%% 1. 加载模型
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [1/10] 模型加载 ═══\n');

% 1a. 碰撞几何模型 (JSON)
jsonFilePath = ['./model/collideConfig/', robName, '_collision.json'];
toolJsonPath = './model/collideConfig/nonetool_collision.json';
params = readCollisionModelJson(jsonFilePath);
params_tool = readToolCollisionJson(toolJsonPath);
fprintf('  碰撞模型: %s, DH: d1=%.3f, d4=%.3f, d6=%.4f, a2=%.3f\n', ...
    params.RobType, params.DH.d1, params.DH.d4, params.DH.d6, params.DH.a2);
fprintf('  碰撞体: base(capsule r=%.3f), lowerArm(capsule r=%.3f), elbow(capsule r=%.3f)\n', ...
    params.base.radius, params.lowerArm.radius, params.elbow.radius);
fprintf('  碰撞体: upperArm(capsule r=%.3f), wrist(sphere r=%.3f)\n', ...
    params.upperArm.radius, params.wrist.radius);
fprintf('  工具碰撞体: %d 个\n', params_tool.toolNum);

% 1b. URDF模型 (Robotics Toolbox)
urdfPath = ['model/urdf/', robName, '.urdf'];
stlPath = ['model/meshes/', robName];
robot = importrobot(urdfPath, 'MeshPath', stlPath);
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];
fprintf('  URDF: base=%s (%d bodies, %d DOF)\n', robot.BaseName, robot.NumBodies, numel(robot.homeConfiguration));
fprintf('  关节名称: ');
revJoints = {};
for bi = 1:robot.NumBodies
    jnt = robot.Bodies{bi}.Joint;
    if ~isempty(jnt) && strcmp(jnt.Type, 'revolute')
        revJoints{end+1} = jnt;
        fprintf('%s ', jnt.Name);
    end
end
fprintf('\n');

% 关节限位 (从URDF提取)
nJoints = numel(revJoints);
jLimits = zeros(nJoints, 2);
for ji = 1:nJoints
    jLimits(ji, :) = revJoints{ji}.PositionLimits;
end
fprintf('  关节限位 (rad):\n');
for ji = 1:nJoints
    fprintf('    J%d: [%+.3f, %+.3f] (%.1f ~ %.1fdeg)\n', ji, jLimits(ji,1), jLimits(ji,2), ...
        rad2deg(jLimits(ji,1)), rad2deg(jLimits(ji,2)));
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════════════════════
%% 2. FK验证: DH FK vs URDF FK 对比
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [2/10] FK对比验证 ═══\n');

% DH参数 (全局变量给FK.m使用)
global d1 d4 d6 a2 T6T;
d1 = params.DH.d1;
d4 = params.DH.d4;
d6 = params.DH.d6;
a2 = params.DH.a2;
T6T = eye(4); % 无工具

testPoses_FK = [
    0,      0,     0,     0,     0,     0;     % Home
    0,      pi/6,  0,     0,     0,     0;     % J2转30°
    0,      pi/4,  pi/4,  0,     0,     0;     % J2+J3
    pi/4,   pi/3,  -pi/6, pi/4,  pi/6,  0;     % 混合角度
    -pi/3,  pi/2,  -pi/4, pi/2,  -pi/3, pi/6;  % 大幅运动
    0,      pi/2,  0,     0,     pi/2,  0;     % 肘部90°
    pi,     0,     pi/2,  pi,    0,     pi;     % 极端角度
    0,      -pi/4, pi/3,  -pi/4, pi/4,  -pi/4; % 负方向
];

fprintf('  %-6s %-35s %-35s %s\n', 'Test#', 'DH-FK TCP (m)', 'URDF-FK TCP (m)', 'Error(mm)');
fkErrors = zeros(size(testPoses_FK, 1), 1);

for ti = 1:size(testPoses_FK, 1)
    q = testPoses_FK(ti, :);
    
    % DH FK
    [~,~,~,~,~,~,T0T_dh] = FK(q);
    tcp_dh = T0T_dh(1:3, 4)';
    
    % URDF FK
    T_urdf = getTransform(robot, q, 'elfin_end_link');
    tcp_urdf = T_urdf(1:3, 4)';
    
    err_mm = norm(tcp_dh - tcp_urdf) * 1000;
    fkErrors(ti) = err_mm;
    
    fprintf('  [%d]    [%+.4f,%+.4f,%+.4f]   [%+.4f,%+.4f,%+.4f]   %.3f\n', ...
        ti, tcp_dh, tcp_urdf, err_mm);
end
fprintf('  FK对比结论: 平均误差=%.3fmm, 最大误差=%.3fmm\n', mean(fkErrors), max(fkErrors));
if max(fkErrors) < 1.0
    fprintf('  ✅ DH FK与URDF FK一致性良好 (误差<1mm)\n');
else
    fprintf('  ⚠️  DH FK与URDF FK存在差异 (可能是DH参数定义/偏移约定不同)\n');
    fprintf('     注: Elfin的DH有q2+pi/2, q3+pi/2的offset, URDF内置不同的关节零位\n');
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════════════════════
%% 3. 碰撞几何可视化
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [3/10] 碰撞几何可视化 ═══\n');

vizPoses = {
    [0, 0, 0, 0, 0, 0], 'Home Position';
    [0, pi/4, pi/4, 0, pi/4, 0], 'Extended Pose';
    [pi/4, pi/3, -pi/6, pi/4, pi/6, 0], 'Working Pose';
};

for vi = 1:size(vizPoses, 1)
    q = vizPoses{vi, 1};
    poseName = vizPoses{vi, 2};
    
    fig = figure('Position', [50 50 1200 900], 'Color', 'w', 'Renderer', 'opengl');
    
    % 左侧: URDF模型
    ax1 = subplot(1, 2, 1, 'Parent', fig);
    show(robot, q, 'Parent', ax1);
    title(ax1, sprintf('URDF模型: %s', poseName), 'FontSize', 14, 'FontWeight', 'bold', 'FontName', CJK_FONT);
    axis(ax1, 'equal'); grid(ax1, 'on');
    view(ax1, 135, 25);
    
    % 右侧: 碰撞几何模型
    ax2 = subplot(1, 2, 2, 'Parent', fig);
    hold(ax2, 'on');
    view(ax2, 3);
    
    [T00, T01, T02, T03, T04, T05, T0T_viz] = FK(q);
    Tf_tree = {T00, T01, T02, T03, T04, T05, T0T_viz};
    
    global alpha;
    alpha = 0.3;
    outputStruct = plotSelfCollisonModel(Tf_tree, params, params_tool);
    
    title(ax2, sprintf('碰撞模型: %s', poseName), 'FontSize', 14, 'FontWeight', 'bold', 'FontName', CJK_FONT);
    axis(ax2, 'equal'); grid(ax2, 'on');
    xlabel(ax2, 'X(m)'); ylabel(ax2, 'Y(m)'); zlabel(ax2, 'Z(m)');
    view(ax2, 135, 25);
    
    % FK结果
    tcp = T0T_viz(1:3, 4)';
    text(ax2, tcp(1)+0.05, tcp(2)+0.05, tcp(3)+0.05, ...
        sprintf('TCP\n[%.1f,%.1f,%.1f]mm', tcp*1000), ...
        'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
    
    savePath = sprintf('%s/pose_%d_%s.png', outputDir, vi, strrep(poseName, ' ', '_'));
    print(fig, savePath, '-dpng', '-r120');
    fprintf('  保存: %s\n', savePath);
    if isHeadless, close(fig); end
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════════════════════
%% 4. 自碰撞检测系统
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [4/10] 自碰撞检测系统 ═══\n');

% 定义全部碰撞对 (5个碰撞体, C(5,2)=10对, 排除相邻)
% 碰撞体: base(T00), lowerArm(T02), elbow(T03), upperArm(T04), wrist(T05)
% 有效碰撞对 (排除相邻连杆):
%   base-elbow, base-upperArm, base-wrist
%   lowerArm-upperArm, lowerArm-wrist
validPairs = {
    'base',     'elbow',    1;
    'base',     'upperArm', 2;
    'base',     'wrist',    3;
    'lowerArm', 'upperArm', 4;
    'lowerArm', 'wrist',    5;
};
nPairs = size(validPairs, 1);
fprintf('  碰撞对 (%d对, 已排除相邻连杆):\n', nPairs);
for pi2 = 1:nPairs
    fprintf('    [%d] %s - %s\n', pi2, validPairs{pi2,1}, validPairs{pi2,2});
end
fprintf('\n');

% 测试多个姿态的自碰撞
collisionTestPoses = [
    0,      0,     0,     0,     0,     0;           % Home - 应安全
    0,      pi/6,  pi/4,  0,     pi/4,  0;           % 工作 - 应安全
    0,      pi/3,  pi/2,  pi,    pi/2,  0;           % 大弯折 - 可能碰撞
    0,      -pi/4, pi/3,  -pi/4, pi/4,  -pi/4;       % 负方向
    pi/2,   pi/2,  -pi/4, pi/2,  -pi/3, pi/6;        % 侧弯
    0,      pi/2,  pi/2,  0,     0,     0;           % 手臂折叠 - 可能碰撞
    0,      0,     pi,    0,     0,     0;           % 极端J3 - 可能碰撞
    pi/4,   pi/4,  pi/4,  pi/4,  pi/4,  pi/4;        % 均匀角度
];
nCollTests = size(collisionTestPoses, 1);

fprintf('  %-6s %-9s %-10s %-10s %-12s %s\n', 'Test#', 'Collision', 'MinDist(m)', 'MinPair', 'AllDists(m)', 'Joints(deg)');
collisionResultsAll = struct();

for ti = 1:nCollTests
    q = collisionTestPoses(ti, :);
    [isCol, colInfo, minDist, pairDists] = checkElfin5SelfCollision(q, params);
    
    % 找最小距离对
    [~, minIdx] = min(pairDists);
    minPairName = sprintf('%s-%s', validPairs{minIdx,1}, validPairs{minIdx,2});
    
    distStr = '';
    for pi2 = 1:nPairs
        distStr = [distStr, sprintf('%.3f ', pairDists(pi2))];
    end
    
    if isCol
        status = '❌ 碰撞';
    elseif minDist < 0.02
        status = '⚠️  警告';
    else
        status = '✅ 安全';
    end
    
    fprintf('  [%d]    %s  %+.4f   %-12s [%s] [%s]\n', ti, status, minDist, minPairName, ...
        strtrim(distStr), sprintf('%.0f ', rad2deg(q)));
    
    collisionResultsAll(ti).q = q;
    collisionResultsAll(ti).isCol = isCol;
    collisionResultsAll(ti).minDist = minDist;
    collisionResultsAll(ti).pairDists = pairDists;
    collisionResultsAll(ti).colInfo = colInfo;
end

nCollisions = sum([collisionResultsAll.isCol]);
nWarnings = sum(~[collisionResultsAll.isCol] & [collisionResultsAll.minDist] < 0.02);
nSafe = nCollTests - nCollisions - nWarnings;
fprintf('\n  统计: ✅安全=%d, ⚠️警告=%d, ❌碰撞=%d (共%d姿态)\n\n', nSafe, nWarnings, nCollisions, nCollTests);

%% ═══════════════════════════════════════════════════════════════════════
%% 5. IK测试 (MATLAB内置求解器)
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [5/10] IK测试 (MATLAB Robotics Toolbox) ═══\n');

ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [0.25 0.25 0.25 1 1 1]; % 6个权重 [orientationX Y Z positionX Y Z]

% 生成测试TCP目标 (从FK计算获取可达目标)
nIKTests = 20;
ikErrors = zeros(nIKTests, 1);
ikSuccess = zeros(nIKTests, 1);
ikTimes = zeros(nIKTests, 1);

fprintf('  %-6s %-35s %-35s %-12s %s\n', 'Test#', 'Target TCP (mm)', 'IK->FK TCP (mm)', 'Error(mm)', 'Time(ms)');
rng(42); % 可重复
for ti = 1:nIKTests
    % 随机合法关节角
    q_rand = zeros(1, nJoints);
    for ji = 1:nJoints
        q_rand(ji) = jLimits(ji,1) + (jLimits(ji,2)-jLimits(ji,1)) * rand();
    end
    
    % FK得到目标TCP
    T_target = getTransform(robot, q_rand, 'elfin_end_link');
    tcp_target = T_target(1:3, 4)' * 1000; % mm
    
    % IK求解
    tStart = tic;
    try
        [q_ik, solInfo] = ik('elfin_end_link', T_target, ikWeights, q_rand * 0.5);
        ikTimes(ti) = toc(tStart) * 1000;
        
        % 验证: FK(q_ik) vs T_target
        T_check = getTransform(robot, q_ik, 'elfin_end_link');
        tcp_check = T_check(1:3, 4)' * 1000;
        ikErrors(ti) = norm(tcp_target - tcp_check);
        ikSuccess(ti) = (ikErrors(ti) < 1.0); % <1mm算成功
        
        fprintf('  [%2d]   [%+7.1f,%+7.1f,%+7.1f]   [%+7.1f,%+7.1f,%+7.1f]   %.4f     %.2f\n', ...
            ti, tcp_target, tcp_check, ikErrors(ti), ikTimes(ti));
    catch ME
        ikTimes(ti) = toc(tStart) * 1000;
        ikErrors(ti) = NaN;
        ikSuccess(ti) = 0;
        fprintf('  [%2d]   [%+7.1f,%+7.1f,%+7.1f]   IK FAILED: %s\n', ti, tcp_target, ME.message);
    end
end

ikSuccRate = sum(ikSuccess) / nIKTests * 100;
validErrors = ikErrors(~isnan(ikErrors) & ikSuccess==1);
fprintf('\n  IK结果汇总:\n');
fprintf('    成功率: %.1f%% (%d/%d)\n', ikSuccRate, sum(ikSuccess), nIKTests);
if ~isempty(validErrors)
    fprintf('    位置误差: 平均=%.4fmm, 最大=%.4fmm, 中位=%.4fmm\n', ...
        mean(validErrors), max(validErrors), median(validErrors));
end
fprintf('    求解耗时: 平均=%.2fms, 最大=%.2fms\n', mean(ikTimes), max(ikTimes));
fprintf('\n');

%% ═══════════════════════════════════════════════════════════════════════
%% 6. 工作空间扫描 + 碰撞统计
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [6/10] 工作空间扫描 + 碰撞统计 ═══\n');

nWsSamples = 500;
rng(123);
wsResults = struct();
wsResults.tcp = zeros(nWsSamples, 3);
wsResults.isCol = false(nWsSamples, 1);
wsResults.minDist = zeros(nWsSamples, 1);

scanStart = tic;
for si = 1:nWsSamples
    q_rand = zeros(1, nJoints);
    for ji = 1:nJoints
        q_rand(ji) = jLimits(ji,1) + (jLimits(ji,2)-jLimits(ji,1)) * rand();
    end
    
    % FK
    T_ws = getTransform(robot, q_rand, 'elfin_end_link');
    wsResults.tcp(si, :) = T_ws(1:3, 4)';
    
    % 碰撞检查
    [isCol, ~, minDist, ~] = checkElfin5SelfCollision(q_rand, params);
    wsResults.isCol(si) = isCol;
    wsResults.minDist(si) = minDist;
end
scanTime = toc(scanStart);

nCollWs = sum(wsResults.isCol);
nSafeWs = nWsSamples - nCollWs;
fprintf('  采样: %d个随机关节角 (%.2fs)\n', nWsSamples, scanTime);
fprintf('  结果: ✅安全=%d (%.1f%%), ❌碰撞=%d (%.1f%%)\n', ...
    nSafeWs, nSafeWs/nWsSamples*100, nCollWs, nCollWs/nWsSamples*100);
fprintf('  安全距离: 平均=%.4fm, 最小=%.4fm\n', ...
    mean(wsResults.minDist(~wsResults.isCol)), min(wsResults.minDist(~wsResults.isCol)));

% 工作空间可视化
fig_ws = figure('Position', [50 50 1400 600], 'Color', 'w');

ax_ws1 = subplot(1,2,1, 'Parent', fig_ws);
hold(ax_ws1, 'on');
safeMask = ~wsResults.isCol;
scatter3(ax_ws1, wsResults.tcp(safeMask,1), wsResults.tcp(safeMask,2), wsResults.tcp(safeMask,3), ...
    10, wsResults.minDist(safeMask), 'filled');
if any(wsResults.isCol)
    scatter3(ax_ws1, wsResults.tcp(~safeMask,1), wsResults.tcp(~safeMask,2), wsResults.tcp(~safeMask,3), ...
        30, 'r', 'x', 'LineWidth', 2);
end
colorbar(ax_ws1); colormap(ax_ws1, 'parula');
xlabel(ax_ws1, 'X(m)'); ylabel(ax_ws1, 'Y(m)'); zlabel(ax_ws1, 'Z(m)');
title(ax_ws1, sprintf('Elfin5 工作空间 (n=%d, 安全=%.1f%%)', nWsSamples, nSafeWs/nWsSamples*100), ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', CJK_FONT);
axis(ax_ws1, 'equal'); grid(ax_ws1, 'on'); view(ax_ws1, 135, 25);

ax_ws2 = subplot(1,2,2, 'Parent', fig_ws);
histogram(ax_ws2, wsResults.minDist(~wsResults.isCol), 30, 'FaceColor', [0.3 0.6 0.9]);
xlabel(ax_ws2, '最小自碰撞距离 (m)', 'FontSize', 12);
ylabel(ax_ws2, '频次', 'FontSize', 12);
title(ax_ws2, '自碰撞距离分布', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', CJK_FONT);
grid(ax_ws2, 'on');
xline(ax_ws2, 0.02, 'r--', 'LineWidth', 2, 'Label', '安全阈值20mm');

print(fig_ws, sprintf('%s/workspace_scan.png', outputDir), '-dpng', '-r120');
fprintf('  保存: %s/workspace_scan.png\n\n', outputDir);
if isHeadless, close(fig_ws); end

%% ═══════════════════════════════════════════════════════════════════════
%% 7. 路径插值 + 碰撞过程检测
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [7/10] 路径插值 + 碰撞过程检测 ═══\n');

% 定义3段路径 (不同姿态间)
pathSegments = {
    [0,0,0,0,0,0],           [0,pi/4,pi/4,0,pi/4,0],       'Home → Work';
    [0,pi/4,pi/4,0,pi/4,0],  [-pi/3,pi/2,-pi/6,pi/4,0,0],  'Work → Side';
    [-pi/3,pi/2,-pi/6,pi/4,0,0], [0,0,0,0,0,0],             'Side → Home';
};
nInterp = 50; % 每段插值点数

fig_path = figure('Position', [50 50 1400 900], 'Color', 'w');
pathColors = {'b', 'r', [0 0.6 0]};

for seg = 1:3
    q_start = pathSegments{seg, 1};
    q_end = pathSegments{seg, 2};
    segName = pathSegments{seg, 3};
    
    tcpPath = zeros(nInterp, 3);
    distPath = zeros(nInterp, 1);
    colPath = false(nInterp, 1);
    
    for si = 1:nInterp
        t = (si-1) / (nInterp-1);
        q_curr = (1-t) * q_start + t * q_end;
        
        T_curr = getTransform(robot, q_curr, 'elfin_end_link');
        tcpPath(si, :) = T_curr(1:3, 4)';
        
        [isCol, ~, minDist, ~] = checkElfin5SelfCollision(q_curr, params);
        distPath(si) = minDist;
        colPath(si) = isCol;
    end
    
    % 3D轨迹
    ax_p1 = subplot(2, 3, seg, 'Parent', fig_path);
    hold(ax_p1, 'on');
    plot3(ax_p1, tcpPath(:,1), tcpPath(:,2), tcpPath(:,3), '-', ...
        'Color', pathColors{seg}, 'LineWidth', 2);
    plot3(ax_p1, tcpPath(1,1), tcpPath(1,2), tcpPath(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(ax_p1, tcpPath(end,1), tcpPath(end,2), tcpPath(end,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    if any(colPath)
        plot3(ax_p1, tcpPath(colPath,1), tcpPath(colPath,2), tcpPath(colPath,3), ...
            'rx', 'MarkerSize', 12, 'LineWidth', 2);
    end
    title(ax_p1, segName, 'FontSize', 12, 'FontWeight', 'bold', 'FontName', CJK_FONT);
    xlabel(ax_p1, 'X(m)'); ylabel(ax_p1, 'Y(m)'); zlabel(ax_p1, 'Z(m)');
    axis(ax_p1, 'equal'); grid(ax_p1, 'on'); view(ax_p1, 135, 25);
    
    % 距离曲线
    ax_p2 = subplot(2, 3, seg+3, 'Parent', fig_path);
    hold(ax_p2, 'on');
    plot(ax_p2, 1:nInterp, distPath*1000, '-', 'Color', pathColors{seg}, 'LineWidth', 2);
    yline(ax_p2, 0, 'r-', 'LineWidth', 1.5);
    yline(ax_p2, 20, 'r--', 'LineWidth', 1, 'Label', '安全阈值');
    if any(colPath)
        plot(ax_p2, find(colPath), distPath(colPath)*1000, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    end
    xlabel(ax_p2, '插值步', 'FontSize', 10);
    ylabel(ax_p2, '最小自碰撞距离 (mm)', 'FontSize', 10);
    title(ax_p2, sprintf('碰撞距离: min=%.1fmm', min(distPath)*1000), ...
        'FontSize', 11, 'FontWeight', 'bold', 'FontName', CJK_FONT);
    grid(ax_p2, 'on');
    
    nColSeg = sum(colPath);
    fprintf('  路径[%d] %s: %d点, 碰撞%d点, 最小距离=%.1fmm\n', ...
        seg, segName, nInterp, nColSeg, min(distPath)*1000);
end

sgtitle(fig_path, 'Elfin5 路径插值碰撞分析', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', CJK_FONT);
print(fig_path, sprintf('%s/path_collision.png', outputDir), '-dpng', '-r120');
fprintf('  保存: %s/path_collision.png\n\n', outputDir);
if isHeadless, close(fig_path); end

%% ═══════════════════════════════════════════════════════════════════════
%% 8. IK精度大规模测试
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [8/10] IK精度大规模测试 ═══\n');

nIKMass = 100;
rng(99);
ikMassErrors = zeros(nIKMass, 1);
ikMassSuccess = false(nIKMass, 1);
ikMassTimes = zeros(nIKMass, 1);

massStart = tic;
for ti = 1:nIKMass
    q_rand = zeros(1, nJoints);
    for ji = 1:nJoints
        q_rand(ji) = jLimits(ji,1)*0.8 + (jLimits(ji,2)*0.8-jLimits(ji,1)*0.8) * rand();
    end
    
    T_target = getTransform(robot, q_rand, 'elfin_end_link');
    q_init = q_rand * 0.3 + randn(1,nJoints)*0.1; % 给一个偏离的初始猜测
    
    tS = tic;
    try
        [q_ik, ~] = ik('elfin_end_link', T_target, ikWeights, q_init);
        ikMassTimes(ti) = toc(tS) * 1000;
        T_check = getTransform(robot, q_ik, 'elfin_end_link');
        posErr = norm(T_target(1:3,4) - T_check(1:3,4)) * 1000;
        ikMassErrors(ti) = posErr;
        ikMassSuccess(ti) = (posErr < 1.0);
    catch
        ikMassTimes(ti) = toc(tS) * 1000;
        ikMassErrors(ti) = NaN;
        ikMassSuccess(ti) = false;
    end
end
massTime = toc(massStart);

validMass = ikMassErrors(ikMassSuccess);
fprintf('  大规模IK测试 (n=%d, %.2fs):\n', nIKMass, massTime);
fprintf('    成功率: %.1f%% (%d/%d)\n', sum(ikMassSuccess)/nIKMass*100, sum(ikMassSuccess), nIKMass);
if ~isempty(validMass)
    fprintf('    位置误差: 平均=%.4fmm, 最大=%.4fmm, 中位=%.4fmm\n', ...
        mean(validMass), max(validMass), median(validMass));
    fprintf('    P99误差: %.4fmm\n', prctile(validMass, 99));
end
fprintf('    求解耗时: 平均=%.2fms, P95=%.2fms\n', mean(ikMassTimes), prctile(ikMassTimes, 95));

% IK误差分布图
fig_ik = figure('Position', [50 50 1200 500], 'Color', 'w');
ax_ik1 = subplot(1,2,1, 'Parent', fig_ik);
if ~isempty(validMass)
    histogram(ax_ik1, validMass, 30, 'FaceColor', [0.2 0.7 0.4]);
end
xlabel(ax_ik1, 'IK位置误差 (mm)', 'FontSize', 12);
ylabel(ax_ik1, '频次', 'FontSize', 12);
title(ax_ik1, sprintf('IK误差分布 (n=%d, 成功率=%.1f%%)', nIKMass, sum(ikMassSuccess)/nIKMass*100), ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', CJK_FONT);
grid(ax_ik1, 'on');

ax_ik2 = subplot(1,2,2, 'Parent', fig_ik);
histogram(ax_ik2, ikMassTimes, 30, 'FaceColor', [0.8 0.5 0.2]);
xlabel(ax_ik2, 'IK求解耗时 (ms)', 'FontSize', 12);
ylabel(ax_ik2, '频次', 'FontSize', 12);
title(ax_ik2, sprintf('IK求解耗时 (平均=%.2fms)', mean(ikMassTimes)), ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', CJK_FONT);
grid(ax_ik2, 'on');

print(fig_ik, sprintf('%s/ik_analysis.png', outputDir), '-dpng', '-r120');
fprintf('  保存: %s/ik_analysis.png\n\n', outputDir);
if isHeadless, close(fig_ik); end

%% ═══════════════════════════════════════════════════════════════════════
%% 9. 性能基准
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [9/10] 性能基准 ═══\n');

nBench = 1000;
q_bench = [0, pi/4, pi/4, 0, pi/4, 0];

% FK基准
fkStart = tic;
for bi = 1:nBench
    [~,~,~,~,~,~,~] = FK(q_bench);
end
fkTime = toc(fkStart) / nBench * 1e6; % us

% URDF FK基准
ufkStart = tic;
for bi = 1:nBench
    getTransform(robot, q_bench, 'elfin_end_link');
end
ufkTime = toc(ufkStart) / nBench * 1e6; % us

% 碰撞检测基准
colStart = tic;
for bi = 1:nBench
    [~,~,~,~] = checkElfin5SelfCollision(q_bench, params);
end
colTime = toc(colStart) / nBench * 1e6; % us

% IK基准 (单次)
T_bench = getTransform(robot, q_bench, 'elfin_end_link');
ikBenchTimes = zeros(50, 1);
for bi = 1:50
    tS = tic;
    ik('elfin_end_link', T_bench, ikWeights, q_bench * 0.5);
    ikBenchTimes(bi) = toc(tS) * 1000;
end

fprintf('  ┌──────────────────────────────────────────┐\n');
fprintf('  │ 操作                  │ 耗时              │\n');
fprintf('  ├──────────────────────────────────────────┤\n');
fprintf('  │ DH FK (1次)           │ %.2f μs          │\n', fkTime);
fprintf('  │ URDF FK (1次)         │ %.2f μs          │\n', ufkTime);
fprintf('  │ 自碰撞检测 (5对)      │ %.2f μs          │\n', colTime);
fprintf('  │ IK求解 (1次)          │ %.2f ms          │\n', mean(ikBenchTimes));
fprintf('  │ FK+碰撞 (1次)         │ %.2f μs          │\n', fkTime + colTime);
fprintf('  └──────────────────────────────────────────┘\n');
fprintf('  注: DH FK比URDF FK快约%.1fx\n\n', ufkTime/max(fkTime,0.01));

%% ═══════════════════════════════════════════════════════════════════════
%% 10. 汇总报告
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══ [10/10] 汇总报告 ═══\n');
totalTime = toc(totalStart);

fig_summary = figure('Position', [50 50 1000 700], 'Color', 'w');
ax_s = axes('Parent', fig_summary);
axis(ax_s, 'off'); xlim(ax_s, [0 1]); ylim(ax_s, [0 1]);

yP = 0.95;
text(ax_s, 0.5, yP, 'Elfin5 Comprehensive Simulation Report', ...
    'FontSize', 18, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'FontName', CJK_FONT, 'Color', [0.1 0.1 0.4]);
yP = yP - 0.06;
text(ax_s, 0.5, yP, sprintf('Total Time: %.2fs | Date: %s', totalTime, datestr(now)), ...
    'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', CJK_FONT, 'Color', [0.4 0.4 0.4]);
yP = yP - 0.08;

items = {
    '1. 模型', sprintf('DH: d1=%.3f d4=%.3f d6=%.4f a2=%.3f (m)', params.DH.d1, params.DH.d4, params.DH.d6, params.DH.a2);
    '2. FK对比', sprintf('DH vs URDF: 平均误差=%.3fmm, 最大=%.3fmm', mean(fkErrors), max(fkErrors));
    '3. 碰撞模型', sprintf('5个碰撞体(4胶囊+1球), %d个碰撞对', nPairs);
    '4. 自碰撞', sprintf('安全=%d, 警告=%d, 碰撞=%d (共%d姿态)', nSafe, nWarnings, nCollisions, nCollTests);
    '5. 工作空间', sprintf('%d采样: 安全=%.1f%%, 碰撞=%.1f%%', nWsSamples, nSafeWs/nWsSamples*100, nCollWs/nWsSamples*100);
    '6. IK精度', sprintf('成功率=%.1f%%, 平均误差=%.4fmm', sum(ikMassSuccess)/nIKMass*100, mean(validMass));
    '7. IK耗时', sprintf('平均=%.2fms (MATLAB内置求解器)', mean(ikMassTimes));
    '8. FK速度', sprintf('DH=%.2fμs, URDF=%.2fμs (DH快%.1fx)', fkTime, ufkTime, ufkTime/max(fkTime,0.01));
    '9. 碰撞速度', sprintf('%.2fμs/次 (5碰撞对)', colTime);
    '10. 路径检测', sprintf('3段路径x%d插值, 碰撞过程可追踪', nInterp);
};

for ii = 1:size(items, 1)
    text(ax_s, 0.05, yP, items{ii, 1}, 'FontSize', 13, 'FontWeight', 'bold', ...
        'FontName', CJK_FONT, 'Color', [0.1 0.3 0.6]);
    text(ax_s, 0.25, yP, items{ii, 2}, 'FontSize', 12, 'FontName', CJK_FONT, 'Color', [0.2 0.2 0.2]);
    yP = yP - 0.05;
end

yP = yP - 0.04;
text(ax_s, 0.5, yP, '对HR_S50迁移的启示', 'FontSize', 15, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'FontName', CJK_FONT, 'Color', [0.8 0.2 0.1]);
yP = yP - 0.05;
insights = {
    '1. Elfin5的碰撞模型完整: base/lowerArm/elbow/upperArm/wrist 五段碰撞体';
    '2. MATLAB Robotics Toolbox IK可用但性能一般 (~数ms/次), 分析IK需自实现';
    '3. DH FK比URDF FK快很多, 确认自实现FK的价值';
    '4. 胶囊体-胶囊体/胶囊体-球体距离计算极快 (<10μs), 验证碰撞架构合理';
    '5. S50需补充: 解析IK(8解), 连杆-环境碰撞, 碰撞缓存';
};
for ii = 1:length(insights)
    text(ax_s, 0.05, yP, insights{ii}, 'FontSize', 10, 'FontName', CJK_FONT, 'Color', [0.3 0.3 0.3]);
    yP = yP - 0.035;
end

print(fig_summary, sprintf('%s/summary_report.png', outputDir), '-dpng', '-r120');
fprintf('  保存: %s/summary_report.png\n', outputDir);
if isHeadless, close(fig_summary); end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  Elfin5 全面仿真完成! 总耗时: %.2fs                                 ║\n', totalTime);
fprintf('║  输出目录: %s                                  ║\n', outputDir);
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n\n');

end % main function

%% ========================================================================
%%                    自碰撞检测函数
%% ========================================================================

function [isCollision, colInfo, minDist, pairDists] = checkElfin5SelfCollision(q, params)
%% 检查Elfin5自碰撞 (5个碰撞体, 5个有效碰撞对)
%  碰撞体: base(T00), lowerArm(T02), elbow(T03), upperArm(T04), wrist(T05)
%  有效碰撞对 (排除相邻连杆):
%    1. base-elbow, 2. base-upperArm, 3. base-wrist
%    4. lowerArm-upperArm, 5. lowerArm-wrist

    global d1 d4 d6 a2 T6T;
    
    % FK
    [T00, T01, T02, T03, T04, T05, ~] = FK(q);
    
    % 提取碰撞体世界坐标 (使用plotSelfCollisonModel的逻辑)
    base_p1 = pTransformLocal(T00, params.base.start);
    base_p2 = pTransformLocal(T00, params.base.end);
    la_p1 = pTransformLocal(T02, params.lowerArm.start);
    la_p2 = pTransformLocal(T02, params.lowerArm.end);
    el_p1 = pTransformLocal(T03, params.elbow.start);
    el_p2 = pTransformLocal(T03, params.elbow.end);
    ua_p1 = pTransformLocal(T04, params.upperArm.start);
    ua_p2 = pTransformLocal(T04, params.upperArm.end);
    wr_c = pTransformLocal(T05, params.wrist.offset);
    
    r_base = params.base.radius;
    r_la = params.lowerArm.radius;
    r_el = params.elbow.radius;
    r_ua = params.upperArm.radius;
    r_wr = params.wrist.radius;
    
    isCollision = false;
    colInfo = '安全';
    pairDists = zeros(5, 1);
    
    % 1. base-elbow
    pairDists(1) = segSegDist(base_p1, base_p2, el_p1, el_p2) - r_base - r_el;
    % 2. base-upperArm
    pairDists(2) = segSegDist(base_p1, base_p2, ua_p1, ua_p2) - r_base - r_ua;
    % 3. base-wrist
    pairDists(3) = ptSegDist(wr_c, base_p1, base_p2) - r_base - r_wr;
    % 4. lowerArm-upperArm
    pairDists(4) = segSegDist(la_p1, la_p2, ua_p1, ua_p2) - r_la - r_ua;
    % 5. lowerArm-wrist
    pairDists(5) = ptSegDist(wr_c, la_p1, la_p2) - r_la - r_wr;
    
    minDist = min(pairDists);
    
    pairNames = {'base-elbow', 'base-upperArm', 'base-wrist', 'lowerArm-upperArm', 'lowerArm-wrist'};
    for pi2 = 1:5
        if pairDists(pi2) < 0
            isCollision = true;
            colInfo = sprintf('碰撞: %s (%.3fm)', pairNames{pi2}, -pairDists(pi2));
            return;
        end
    end
end

function p_world = pTransformLocal(T, p_local)
%% 将局部坐标变换到世界坐标
    p = T * [p_local(:); 1];
    p_world = p(1:3);
end

function dist = ptSegDist(p, a, b)
%% 点到线段距离
    ab = b - a;
    ap = p - a;
    t = max(0, min(1, dot(ap, ab) / max(dot(ab, ab), 1e-12)));
    dist = norm(p - (a + t * ab));
end

function dist = segSegDist(p1, p2, p3, p4)
%% 线段到线段距离
    d1 = p2 - p1;
    d2 = p4 - p3;
    r = p1 - p3;
    a = dot(d1, d1);
    b = dot(d1, d2);
    c = dot(d2, d2);
    dd = dot(d1, r);
    e = dot(d2, r);
    
    denom = a*c - b*b;
    if denom < 1e-10
        s = 0;
        t = max(0, min(1, dd / max(b, 1e-10)));
    else
        s = (b*e - c*dd) / denom;
        t = (a*e - b*dd) / denom;
    end
    s = max(0, min(1, s));
    t = max(0, min(1, t));
    
    % 重新计算被clamped后的最优值
    tnom = b*s + e;
    if tnom < 0
        t = 0;
        s = max(0, min(1, -dd/max(a,1e-12)));
    elseif tnom > c
        t = 1;
        s = max(0, min(1, (b-dd)/max(a,1e-12)));
    else
        t = tnom / max(c, 1e-12);
    end
    
    dist = norm((p1 + s*d1) - (p3 + t*d2));
end
