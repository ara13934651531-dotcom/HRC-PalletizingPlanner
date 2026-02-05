%% testS50.m - HR_S50-2000 协作机器人碰撞检测与3D可视化仿真
% 
% 功能：
%   1. 加载S50机器人碰撞几何模型
%   2. 测试不同关节配置下的碰撞检测
%   3. 3D可视化显示碰撞状态（红色=碰撞，绿色=安全）
%   4. 支持动态演示碰撞过程
%
% HR_S50-2000 DH参数 (mm):
%   d1=296.5, d2=336.2, d3=239.0, d4=158.5, d5=158.5, d6=134.5
%   a2=900.0, a3=941.5
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
% Author: Huayan Robotics
% Website: https://www.huayan-robotics.com

close all;
clear all;
clc;

addpath collisionVisual\
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
robName = 'S50';  % HR_S50-2000 机器人

% 测试多组关节配置 (单位: rad)
% 基于HR_S50-2000关节限位: J1:±360°, J2:-190°~+10°, J3:±165°, J4-J6:±360°
testCases = {
    % 测试1: 零位姿态 (安全)
    struct('name', '零位姿态', ...
           'joints', [0, 0, 0, 0, 0, 0], ...
           'description', '所有关节归零，机器人完全伸展');
    
    % 测试2: 常规工作姿态 (安全)
    struct('name', '常规工作姿态', ...
           'joints', [0, -pi/2, pi/6, 0, -pi/3, 0], ...
           'description', '典型码垛作业姿态');
    
    % 测试3: 手臂折叠姿态 - 可能发生自碰撞
    struct('name', '折叠姿态(潜在碰撞)', ...
           'joints', [0, pi/12, -pi*3/4, pi, 0, 0], ...
           'description', '手臂向基座方向折叠');
    
    % 测试4: 极限弯曲 - 高碰撞风险
    struct('name', '极限弯曲(高风险)', ...
           'joints', [pi/2, -pi/2, -pi/2, -pi/4, pi/2, 0], ...
           'description', '多关节同时弯曲，碰撞风险较高');
    
    % 测试5: 工作空间边界
    struct('name', '工作空间边界', ...
           'joints', [pi/3, -pi/3, pi/4, pi/6, -pi/4, pi/6], ...
           'description', '接近工作空间边界的配置');
    
    % 测试6: 自碰撞演示
    struct('name', '自碰撞配置', ...
           'joints', [0, 0, -2.5, pi, 0, 0], ...
           'description', '刻意制造的自碰撞配置');
};

%% ====================== 主程序 ======================
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   HR_S50-2000 碰撞检测与3D可视化仿真                    ║\n');
fprintf('║   Huayan Robotics - https://www.huayan-robotics.com     ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

% 读取碰撞模型配置
jsonFilePath = "./model/collideConfig/S50_collision.json";
tooljsonFilePath = "./model/collideConfig/nonetool_collision.json";
jsonSavePath = ['./data/', robName, '_output.json'];

fprintf('📂 加载碰撞模型配置...\n');
params = readCollisionModelJson(jsonFilePath);
params_tool = readToolCollisionJson(tooljsonFilePath);
fprintf('✅ 配置加载完成\n\n');

% 显示DH参数
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('📐 HR_S50-2000 DH参数 (单位: m)\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   d1 = %.4f    d2 = %.4f    d3 = %.4f\n', params.DH.d1, params.DH.d2, params.DH.d3);
fprintf('   d4 = %.4f    d5 = %.4f    d6 = %.4f\n', params.DH.d4, params.DH.d5, params.DH.d6);
fprintf('   a2 = %.4f    a3 = %.4f\n', params.DH.a2, params.DH.a3);
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% 遍历测试用例
numTests = length(testCases);
collisionResults = cell(numTests, 1);

for testIdx = 1:numTests
    testCase = testCases{testIdx};
    jointPositions = testCase.joints;
    
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('🔍 测试 %d/%d: %s\n', testIdx, numTests, testCase.name);
    fprintf('   描述: %s\n', testCase.description);
    fprintf('   关节角度 (deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', ...
        jointPositions(1)*180/pi, jointPositions(2)*180/pi, ...
        jointPositions(3)*180/pi, jointPositions(4)*180/pi, ...
        jointPositions(5)*180/pi, jointPositions(6)*180/pi);
    
    % 创建新图形窗口
    fig = figure('Name', sprintf('S50 测试%d: %s', testIdx, testCase.name), ...
           'Position', [100 + (testIdx-1)*50, 100 + (testIdx-1)*50, 1000, 800], ...
           'Color', 'white');
    
    % 设置全局透明度
    global alpha;
    alpha = 0.4;
    
    % 进行碰撞几何仿真
    outputStruct = plotS50CollisionModel(jointPositions, params, params_tool);
    
    % 碰撞检测
    [isCollision, collisionInfo, minDist] = checkS50SelfCollision(outputStruct, params);
    collisionResults{testIdx} = struct('name', testCase.name, ...
                                        'joints', jointPositions, ...
                                        'isCollision', isCollision, ...
                                        'info', collisionInfo, ...
                                        'minDistance', minDist);
    
    % 设置标题和视角
    if isCollision
        titleStr = sprintf('HR\\_S50-2000: %s\\n⚠️ 检测到碰撞! %s', testCase.name, collisionInfo);
        titleColor = [0.8 0 0];
        fprintf('   ⚠️  结果: 检测到碰撞! %s\n', collisionInfo);
    else
        titleStr = sprintf('HR\\_S50-2000: %s\\n✅ 安全 (最小距离: %.3fm)', testCase.name, minDist);
        titleColor = [0 0.6 0];
        fprintf('   ✅ 结果: 无碰撞，安全 (最小距离: %.3fm)\n', minDist);
    end
    
    title(titleStr, 'FontSize', 13, 'Color', titleColor, 'FontWeight', 'bold', 'Interpreter', 'tex');
    view(135, 25);
    axis equal;
    grid on;
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    zlabel('Z (m)', 'FontSize', 12);
    
    % 设置坐标轴范围
    xlim([-1.5, 1.5]);
    ylim([-1.5, 1.5]);
    zlim([-0.5, 2.0]);
    
    % 添加光照效果
    camlight('headlight');
    lighting gouraud;
    
    drawnow;

    % 头less环境保存图像
    if isHeadless
        imgName = sprintf('%s/S50_test_%02d.png', outputDir, testIdx);
        saveas(fig, imgName);
        close(fig);
    end
end

%% 汇总结果
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║                    碰撞检测结果汇总                      ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');

collisionCount = 0;
for i = 1:numTests
    result = collisionResults{i};
    if result.isCollision
        status = '碰撞';
        collisionCount = collisionCount + 1;
    else
        status = '安全';
    end
    fprintf('║ %d. %-20s   %s   (%.3fm) ║\n', i, result.name, status, result.minDistance);
end

fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║ 总计: %d/%d 配置检测到碰撞                              ║\n', collisionCount, numTests);
fprintf('╚══════════════════════════════════════════════════════════╝\n');

%% 保存最后一个测试用例的数据
saveUIdata(testCases{end}.joints, outputStruct, params, jsonSavePath);
fprintf('\n数据已保存至: %s\n', jsonSavePath);

fprintf('\n仿真完成!\n');
fprintf('提示: 运行 testS50_Dynamic.m 可查看动态碰撞演示\n');

if isHeadless
    fprintf('无图形界面：已保存结果图像至 %s\n', outputDir);
end


%% ====================== 辅助函数 ======================

function outputStruct = plotS50CollisionModel(q, params, toolparams)
    %% 绘制S50碰撞模型
    global T6T;
    global d1 d2 d3 d4 d5 d6 a2 a3;
    
    % 设置DH参数
    d1 = params.DH.d1; 
    d2 = params.DH.d2;
    d3 = params.DH.d3;
    d4 = params.DH.d4; 
    d5 = params.DH.d5; 
    d6 = params.DH.d6; 
    a2 = -params.DH.a2;  % 注意: 负号
    a3 = -params.DH.a3;  % 注意: 负号

    % 工具坐标系
    T6T = eye(4);
    
    % 计算正运动学
    [T00, T01, T02, T03, T04, T05, T0T] = FK_SSerial(q);
    Tf_tree = {T00, T01, T02, T03, T04, T05, T0T};

    % 绘制坐标系
    for i = 1:length(Tf_tree)
        plotframe(Tf_tree{i}, 0.1, true);
        position = Tf_tree{i}(1:3, 4);
        text(position(1), position(2), position(3) + 0.08, ...
             ['T' num2str(i-1)], 'FontSize', 10, 'Color', 'blue', 'FontWeight', 'bold');
    end
    
    % 绘制碰撞几何
    outputStruct = plotSelfCollisonModel(Tf_tree, params, toolparams);
end

function [isCollision, collisionInfo, minDistance] = checkS50SelfCollision(outputStruct, params)
    %% 检查S50自碰撞 - 基于胶囊体和球体的几何碰撞检测
    % 
    % 碰撞检测逻辑：
    %   1. 计算各碰撞体之间的最小距离
    %   2. 如果距离小于两个碰撞体半径之和，则判定为碰撞
    %   3. 只检测非相邻连杆之间的碰撞
    
    isCollision = false;
    collisionInfo = '';
    minDistance = inf;
    
    % 获取各部分的端点位置 (胶囊体用两端点表示，球体用中心表示)
    base_p1 = outputStruct.base_bc1(:);
    base_p2 = outputStruct.base_bc2(:);
    lowerArm_p1 = outputStruct.lowerArm_la1(:);
    lowerArm_p2 = outputStruct.lowerArm_la2(:);
    elbow_p1 = outputStruct.elbow_e1(:);
    elbow_p2 = outputStruct.elbow_e2(:);
    upperArm_p1 = outputStruct.upperArm_ua1(:);
    upperArm_p2 = outputStruct.upperArm_ua2(:);
    wrist_center = outputStruct.wrist_wc(:);
    
    % 碰撞体半径
    r_base = params.base.radius;
    r_lowerArm = params.lowerArm.radius;
    r_elbow = params.elbow.radius;
    r_upperArm = params.upperArm.radius;
    r_wrist = params.wrist.radius;
    
    % 定义需要检测的碰撞对 (只检测非相邻连杆)
    % 碰撞对格式: {名称, 体1类型, 体1数据, 半径1, 体2类型, 体2数据, 半径2}
    collisionPairs = {
        % 基座 vs 肘部
        {'基座-肘部', 'capsule', [base_p1, base_p2], r_base, 'capsule', [elbow_p1, elbow_p2], r_elbow};
        % 基座 vs 上臂
        {'基座-上臂', 'capsule', [base_p1, base_p2], r_base, 'capsule', [upperArm_p1, upperArm_p2], r_upperArm};
        % 基座 vs 腕部
        {'基座-腕部', 'capsule', [base_p1, base_p2], r_base, 'sphere', wrist_center, r_wrist};
        % 下臂 vs 上臂
        {'下臂-上臂', 'capsule', [lowerArm_p1, lowerArm_p2], r_lowerArm, 'capsule', [upperArm_p1, upperArm_p2], r_upperArm};
        % 下臂 vs 腕部
        {'下臂-腕部', 'capsule', [lowerArm_p1, lowerArm_p2], r_lowerArm, 'sphere', wrist_center, r_wrist};
    };
    
    % 遍历所有碰撞对进行检测
    for i = 1:length(collisionPairs)
        pair = collisionPairs{i};
        pairName = pair{1};
        type1 = pair{2};
        data1 = pair{3};
        radius1 = pair{4};
        type2 = pair{5};
        data2 = pair{6};
        radius2 = pair{7};
        
        % 计算两个碰撞体之间的距离
        dist = computeCollisionDistance(type1, data1, type2, data2);
        
        % 计算净距离 (表面间距离)
        netDist = dist - radius1 - radius2;
        
        % 更新最小距离
        if netDist < minDistance
            minDistance = netDist;
        end
        
        % 检测碰撞
        if netDist < 0
            isCollision = true;
            collisionInfo = sprintf('%s (穿透深度: %.3fm)', pairName, -netDist);
            return;
        end
    end
    
    if ~isCollision
        collisionInfo = '所有检查通过';
    end
end

function dist = computeCollisionDistance(type1, data1, type2, data2)
    %% 计算两个碰撞体之间的最小距离
    
    if strcmp(type1, 'capsule') && strcmp(type2, 'capsule')
        % 胶囊体 vs 胶囊体: 线段间最小距离
        p1 = data1(:,1);
        p2 = data1(:,2);
        p3 = data2(:,1);
        p4 = data2(:,2);
        dist = segmentToSegmentDistance(p1, p2, p3, p4);
        
    elseif strcmp(type1, 'capsule') && strcmp(type2, 'sphere')
        % 胶囊体 vs 球体: 点到线段距离
        p1 = data1(:,1);
        p2 = data1(:,2);
        center = data2;
        dist = pointToSegmentDistance(center, p1, p2);
        
    elseif strcmp(type1, 'sphere') && strcmp(type2, 'capsule')
        % 球体 vs 胶囊体
        center = data1;
        p1 = data2(:,1);
        p2 = data2(:,2);
        dist = pointToSegmentDistance(center, p1, p2);
        
    elseif strcmp(type1, 'sphere') && strcmp(type2, 'sphere')
        % 球体 vs 球体: 中心距离
        dist = norm(data1 - data2);
    else
        dist = inf;
    end
end

function dist = pointToSegmentDistance(p, a, b)
    %% 计算点到线段的最短距离
    ab = b - a;
    ap = p - a;
    
    t = dot(ap, ab) / dot(ab, ab);
    t = max(0, min(1, t));  % 限制在[0,1]范围内
    
    closest = a + t * ab;
    dist = norm(p - closest);
end

function dist = segmentToSegmentDistance(p1, p2, p3, p4)
    %% 计算两条线段之间的最短距离
    % 使用参数化方法求解
    
    d1 = p2 - p1;  % 线段1方向
    d2 = p4 - p3;  % 线段2方向
    r = p1 - p3;
    
    a = dot(d1, d1);
    b = dot(d1, d2);
    c = dot(d2, d2);
    d = dot(d1, r);
    e = dot(d2, r);
    
    denom = a*c - b*b;
    
    if denom < 1e-10
        % 平行线段
        s = 0;
        t = d / b;
    else
        s = (b*e - c*d) / denom;
        t = (a*e - b*d) / denom;
    end
    
    % 限制参数在[0,1]范围内
    s = max(0, min(1, s));
    t = max(0, min(1, t));
    
    % 重新计算以确保最近点
    if s < 0 || s > 1
        s = max(0, min(1, s));
        t = (b*s + e) / c;
        t = max(0, min(1, t));
    end
    
    if t < 0 || t > 1
        t = max(0, min(1, t));
        s = (b*t - d) / a;
        s = max(0, min(1, s));
    end
    
    closest1 = p1 + s * d1;
    closest2 = p3 + t * d2;
    dist = norm(closest1 - closest2);
end
