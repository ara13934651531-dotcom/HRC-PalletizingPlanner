%% testS50_Dynamic.m - HR_S50-2000 动态碰撞演示
% 
% 功能：
%   1. 动态演示机器人从安全姿态移动到碰撞姿态
%   2. 实时显示碰撞状态变化
%   3. 不同颜色标识安全/碰撞状态
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
% Author: Huayan Robotics

close all;
clear all;
clc;

addpath collisionVisual/
addcollisionVisualPath;

%% ====================== 运行环境设置 ======================
isHeadless = ~usejava('desktop');
outputDir = './pic/S50_sim';
if isHeadless
    set(0, 'DefaultFigureVisible', 'off');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

%% ====================== 配置参数 ======================
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   HR_S50-2000 动态碰撞演示                              ║\n');
fprintf('║   Huayan Robotics - https://www.huayan-robotics.com     ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

% 读取碰撞模型配置
jsonFilePath = "./model/collideConfig/S50_collision.json";
tooljsonFilePath = "./model/collideConfig/nonetool_collision.json";

fprintf('📂 加载碰撞模型配置...\n');
params = readCollisionModelJson(jsonFilePath);
params_tool = readToolCollisionJson(tooljsonFilePath);
fprintf('✅ 配置加载完成\n\n');

%% 定义起始和目标关节配置
% 起始: 安全的展开姿态
startJoints = [0, -pi/4, pi/6, 0, 0, 0];
% 目标: 可能碰撞的折叠姿态
endJoints = [0, 0, -2.5, pi, 0, 0];

% 动画帧数
numFrames = 60;

%% 创建图形窗口
fig = figure('Name', 'HR_S50-2000 动态碰撞演示', ...
       'Position', [100, 100, 1200, 800], ...
       'Color', 'white');

% 创建子图布局
subplot(1, 2, 1);
ax1 = gca;
title('3D 碰撞可视化', 'FontSize', 14);

subplot(1, 2, 2);
ax2 = gca;
title('碰撞状态监控', 'FontSize', 14);

%% 动画主循环
fprintf('🎬 开始动态演示...\n');
fprintf('   从安全姿态过渡到碰撞姿态\n\n');

collisionHistory = [];
distanceHistory = [];
frameIdx = [];

for i = 1:numFrames
    % 计算当前帧的关节角度 (线性插值)
    t = (i - 1) / (numFrames - 1);
    currentJoints = startJoints + t * (endJoints - startJoints);
    
    % 清除3D视图
    subplot(1, 2, 1);
    cla;
    hold on;
    
    % 设置全局透明度
    global alpha;
    alpha = 0.5;
    
    % 绘制碰撞模型
    outputStruct = plotS50CollisionModelDynamic(currentJoints, params, params_tool);
    
    % 碰撞检测
    [isCollision, collisionInfo, minDist] = checkS50Collision(outputStruct, params);
    
    % 记录历史数据
    collisionHistory(i) = isCollision;
    distanceHistory(i) = minDist;
    frameIdx(i) = i;
    
    % 设置3D视图属性
    if isCollision
        titleColor = [0.9 0 0];
        statusStr = sprintf('⚠️ 碰撞! %s', collisionInfo);
    else
        titleColor = [0 0.6 0];
        statusStr = sprintf('✅ 安全 (距离: %.3fm)', minDist);
    end
    
    title(sprintf('帧 %d/%d - %s', i, numFrames, statusStr), ...
          'FontSize', 12, 'Color', titleColor);
    view(135, 25);
    axis equal;
    grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    xlim([-1.5, 1.5]); ylim([-1.5, 1.5]); zlim([-0.5, 2.0]);
    
    % 更新状态监控图
    subplot(1, 2, 2);
    cla;
    hold on;
    
    % 绘制距离曲线
    yyaxis left;
    plot(frameIdx, distanceHistory, 'b-', 'LineWidth', 2);
    ylabel('最小距离 (m)', 'FontSize', 11);
    ylim([-0.2, 0.5]);
    
    % 绘制安全阈值线
    yline(0, 'r--', 'LineWidth', 1.5, 'Label', '碰撞阈值');
    
    % 标记碰撞区域
    collisionFrames = find(collisionHistory);
    if ~isempty(collisionFrames)
        for cf = collisionFrames
            xline(cf, 'r-', 'LineWidth', 1, 'Alpha', 0.3);
        end
    end
    
    % 绘制关节角度
    yyaxis right;
    plot(frameIdx, arrayfun(@(x) startJoints(3) + x/numFrames*(endJoints(3)-startJoints(3)), frameIdx), ...
         'g--', 'LineWidth', 1.5);
    ylabel('关节3角度 (rad)', 'FontSize', 11);
    
    xlabel('帧数', 'FontSize', 11);
    title('碰撞状态监控', 'FontSize', 12);
    legend('最小距离', '碰撞阈值', '关节3角度', 'Location', 'northeast');
    grid on;
    xlim([1, numFrames]);
    
    % 显示当前关节角度
    text(0.02, 0.98, sprintf('关节角度 (deg):\n[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]', ...
        currentJoints(1)*180/pi, currentJoints(2)*180/pi, ...
        currentJoints(3)*180/pi, currentJoints(4)*180/pi, ...
        currentJoints(5)*180/pi, currentJoints(6)*180/pi), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontSize', 9, 'BackgroundColor', [1 1 1 0.8]);
    
    drawnow;
    if ~isHeadless
        pause(0.05);  % 控制动画速度
    end
end

fprintf('🎉 动态演示完成!\n');
fprintf('   碰撞发生在帧 %d\n', find(collisionHistory, 1));

if isHeadless
    imgName = sprintf('%s/S50_dynamic_summary.png', outputDir);
    saveas(fig, imgName);
    close(fig);
    fprintf('无图形界面：已保存动态演示图至 %s\n', imgName);
end

%% ====================== 辅助函数 ======================

function outputStruct = plotS50CollisionModelDynamic(q, params, toolparams)
    %% 绘制S50碰撞模型 (动态版本，无坐标系标注)
    global T6T;
    global d1 d2 d3 d4 d5 d6 a2 a3;
    
    d1 = params.DH.d1; 
    d2 = params.DH.d2;
    d3 = params.DH.d3;
    d4 = params.DH.d4; 
    d5 = params.DH.d5; 
    d6 = params.DH.d6; 
    a2 = -params.DH.a2;
    a3 = -params.DH.a3;

    T6T = eye(4);
    
    [T00, T01, T02, T03, T04, T05, T0T] = FK_SSerial(q);
    Tf_tree = {T00, T01, T02, T03, T04, T05, T0T};
    
    outputStruct = plotSelfCollisonModel(Tf_tree, params, toolparams);
end

function [isCollision, collisionInfo, minDistance] = checkS50Collision(outputStruct, params)
    %% 检查S50自碰撞
    isCollision = false;
    collisionInfo = '';
    minDistance = inf;
    
    base_p1 = outputStruct.base_bc1(:);
    base_p2 = outputStruct.base_bc2(:);
    elbow_p1 = outputStruct.elbow_e1(:);
    elbow_p2 = outputStruct.elbow_e2(:);
    upperArm_p1 = outputStruct.upperArm_ua1(:);
    upperArm_p2 = outputStruct.upperArm_ua2(:);
    wrist_center = outputStruct.wrist_wc(:);
    
    r_base = params.base.radius;
    r_elbow = params.elbow.radius;
    r_upperArm = params.upperArm.radius;
    r_wrist = params.wrist.radius;
    
    % 基座 vs 肘部
    dist1 = segDistToSeg(base_p1, base_p2, elbow_p1, elbow_p2) - r_base - r_elbow;
    if dist1 < minDistance, minDistance = dist1; end
    if dist1 < 0
        isCollision = true;
        collisionInfo = sprintf('基座-肘部 (%.3fm)', -dist1);
        return;
    end
    
    % 基座 vs 上臂
    dist2 = segDistToSeg(base_p1, base_p2, upperArm_p1, upperArm_p2) - r_base - r_upperArm;
    if dist2 < minDistance, minDistance = dist2; end
    if dist2 < 0
        isCollision = true;
        collisionInfo = sprintf('基座-上臂 (%.3fm)', -dist2);
        return;
    end
    
    % 基座 vs 腕部
    dist3 = ptToSeg(wrist_center, base_p1, base_p2) - r_base - r_wrist;
    if dist3 < minDistance, minDistance = dist3; end
    if dist3 < 0
        isCollision = true;
        collisionInfo = sprintf('基座-腕部 (%.3fm)', -dist3);
        return;
    end
    
    collisionInfo = '安全';
end

function d = ptToSeg(p, a, b)
    ab = b - a;
    t = max(0, min(1, dot(p-a, ab) / dot(ab, ab)));
    d = norm(p - (a + t*ab));
end

function d = segDistToSeg(p1, p2, p3, p4)
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
        s = 0; t = dd/b;
    else
        s = (b*e - c*dd) / denom;
        t = (a*e - b*dd) / denom;
    end
    s = max(0, min(1, s));
    t = max(0, min(1, t));
    
    d = norm((p1 + s*d1) - (p3 + t*d2));
end
