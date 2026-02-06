%% testS50_Palletizing.m - HR_S50-2000 码垛工作站场景仿真
% 
% 严格按照实物照片(2026.01.29)配置的码垛场景：
%   - 机械臂基座下方：白色电箱（控制柜）
%   - 机械臂正前方（面向传送带时的左侧）：蓝色码垛框架
%   - 机械臂右侧：传送带（带箱子）
%   - 末端执行器：水平吸盘，对准箱子顶部中心
%
% 碰撞检测场景：
%   1. 机械臂本体与蓝色码垛框的碰撞
%   2. 末端执行器与传送带/已堆积箱子的碰撞
%   3. 机械臂本体自碰撞（臂展长达2m）
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

addpath('collisionVisual');
addpath(genpath('collisionVisual'));

%% ====================== 运行环境设置 ======================
isHeadless = ~usejava('desktop');
outputDir = './pic/S50_palletizing';
if isHeadless
    set(0, 'DefaultFigureVisible', 'off');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

%% ====================== 场景参数配置（严格按照实物） ======================
% 单位：米 (m)
%
% 实物照片布局分析 (2026.01.29):
%   - 观察视角：站在控制柜正面观察
%   - 机械臂基座：安装在控制柜顶部中央
%   - 蓝色码垛框：在机械臂正前方偏左（-X,-Y方向）
%   - 传送带：在机械臂右侧（+Y方向）
%   - 控制柜：白色，正面有显示屏和急停按钮
%
% 坐标系定义（按实物朝向）：
%   +X: 控制柜后方
%   +Y: 控制柜右侧（传送带方向）
%   +Z: 向上

% 电箱/控制柜参数 (机械臂基座下方)
cabinet.width = 0.55;    % 宽度 X方向 (加厚以匹配实物)
cabinet.depth = 0.65;    % 深度 Y方向  
cabinet.height = 0.80;   % 高度 Z方向（平台高度，稍高）
cabinet.color = [0.95, 0.95, 0.93];  % 白灰色

% 蓝色码垛框架参数 (机械臂左侧，-Y方向)
% 根据实物照片：框架主体约1.2m宽×1.0m深×2.0m高
% 增大距离以避免碰撞
frame.width = 1.20;      % 框架宽度 (Y方向)
frame.depth = 1.00;      % 框架深度 (X方向)
frame.height = 2.00;     % 框架高度 (Z方向)
frame.tubeRadius = 0.040; % 管材半径（主结构）
frame.posX = 0.0;        % 框架中心X位置（与机械臂对齐）
frame.posY = -1.30;      % 框架中心Y位置（增大距离，避免碰撞）
frame.posZ = 0;          % 框架底部在地面
frame.color = [0.25, 0.55, 0.85];  % 蓝色（更鲜艳）

% 传送带参数 (机械臂右侧，+Y方向)
% 关键：传送带在机械臂右侧（不是后方），与码垛框架相对
conveyor.length = 1.80;  % 传送带长度 (X方向)
conveyor.width = 0.50;   % 传送带宽度 (Y方向)
conveyor.height = 0.75;  % 传送带高度 (从地面，略低于平台)
conveyor.posX = 0.10;    % 传送带中心X (稍偏前)
conveyor.posY = 1.35;    % 传送带中心Y（机械臂右侧，增大距离）
conveyor.beltHeight = 0.04; % 皮带厚度
conveyor.color = [0.30, 0.30, 0.32];  % 深灰色

% 箱子参数 (传送带上) - 按实物棕色纸箱
box.length = 0.40;       % 箱子长度 (X)
box.width = 0.30;        % 箱子宽度 (Y)
box.height = 0.25;       % 箱子高度 (Z)
box.color = [0.65, 0.45, 0.25];  % 棕色纸箱（更真实）

%% ====================== 关键位姿定义（码垛作业） ======================
% 机械臂初始朝向：面向传送带方向（+Y方向）
% 所有姿态都要保证末端执行器水平（吸盘朝下）

% 传送带上箱子的抓取位置（TCP对准箱子顶部中心）
boxOnConveyor.x = conveyor.posX;
boxOnConveyor.y = conveyor.posY - conveyor.width/4;
boxOnConveyor.z = conveyor.height + conveyor.beltHeight + box.height;  % 箱子顶部（精确对齐）

% 码垛框内放置位置（第一层，TCP对准箱子顶部）
palletPlace.x = frame.posX + 0.15;  % 框架内侧
palletPlace.y = frame.posY + frame.width/4;
palletPlace.z = frame.posZ + box.height;  % 第一层箱子顶部

% 码垛关键姿态（单位：弧度）
%
% S-Serial机器人末端朝下的关键发现：
%   q=[0,-90,90,-90,-90,0] → 末端Z轴=[0,0,-1] 垂直朝下！
%   
% 分析：
%   - q2=-90°: 大臂水平
%   - q3=+90°: 小臂垂直向下折
%   - q4=-90°: 腕部旋转90°
%   - q5=-90°: 腕部俯仰使末端垂直
%
% J1控制方向：
%   - J1=-90°: 机械臂朝+Y（传送带）
%   - J1=+90°: 机械臂朝-Y（码垛框）
%
% 实物场景布局：
%   - 传送带: +Y方向 (Y≈0.7~1.2m), 高度0.75m
%   - 码垛框: -Y方向 (Y≈-0.3~-1.5m), 地面起
%   - 箱子高度0.25m

poses = {
    % 1. 初始/等待姿态 - 安全位置，收回，面向传送带
    struct('name', '初始等待', ...
           'joints', [-pi/2, -pi/2, pi/3, 0, -pi/3, 0], ...
           'description', '安全等待，面向传送带(+Y)');
    
    % 2. 传送带上方 - 末端垂直朝下，TCP对准箱子中心
    % q2=-90°, q3=90°, q4=-90°, q5=-90° → 末端Z=[0,0,-1]
    struct('name', '传送带上方', ...
           'joints', [-pi/2, -pi/2, pi/2, -pi/2, -pi/2, 0], ...
           'description', '传送带上方，TCP垂直对准箱子');
    
    % 3. 抓取箱子 - TCP贴合箱子顶部
    % 实际应用中通过笛卡尔直线插补实现精确下降
    struct('name', '抓取箱子', ...
           'joints', [-pi/2, -pi/2, 19*pi/30, -pi/2, -pi/2, 0], ...
           'description', 'TCP贴合箱子顶部，准备抓取');
    
    % 4. 提升箱子 - 保持末端垂直
    struct('name', '提升箱子', ...
           'joints', [-pi/2, -pi/2, pi/2, -pi/2, -pi/2, 0], ...
           'description', '垂直提升，TCP保持朝下');
    
    % 5. 转向码垛区 (J1从-90°到-180°，避免大幅度旋转)
    struct('name', '转向码垛', ...
           'joints', [-pi, -pi/2, pi/2, -pi/2, -pi/2, 0], ...
           'description', '旋转J1，转向码垛框(-Y)');
    
    % 6. 码垛区上方 - 末端垂直，准备放置
    struct('name', '码垛上方', ...
           'joints', [-pi, -pi/2, pi/2, -pi/2, -pi/2, 0], ...
           'description', '码垛框上方，TCP垂直');
    
    % 7. 放置箱子 - TCP贴合目标位置
    struct('name', '放置箱子', ...
           'joints', [-pi, -pi/2, 19*pi/30, -pi/2, -pi/2, 0], ...
           'description', 'TCP下降到放置位置');
    
    % 8. 释放后抬起 - 保持垂直
    struct('name', '释放抬起', ...
           'joints', [-pi, -pi/2, pi/2, -pi/2, -pi/2, 0], ...
           'description', '释放后垂直抬起');
    
    % 9. 返回传送带方向 - 末端垂直
    struct('name', '返回传送带', ...
           'joints', [-pi/2, -pi/2, pi/2, -pi/2, -pi/2, 0], ...
           'description', '转回传送带方向');
    
    % 10. 回到初始位置
    struct('name', '回到初始', ...
           'joints', [-pi/2, -pi/2, pi/3, 0, -pi/3, 0], ...
           'description', '完成循环，回到等待');
};

%% ====================== 主程序 ======================
fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║     HR_S50-2000 码垛工作站场景仿真 (按实物照片配置)             ║\n');
fprintf('║     Huayan Robotics - https://www.huayan-robotics.com           ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');

fprintf('📷 场景配置（严格按照实物照片 2026.01.29）:\n');
fprintf('   • 电箱/控制柜: 机械臂基座下方, 高度 %.2fm\n', cabinet.height);
fprintf('   • 蓝色码垛框: 机械臂左侧, 位置 (%.2f, %.2f)\n', frame.posX, frame.posY);
fprintf('   • 传送带: 机械臂右侧 (+Y方向), 位置 Y=%.2fm\n', conveyor.posY);
fprintf('   • 箱子尺寸: %.0f×%.0f×%.0fcm\n\n', box.length*100, box.width*100, box.height*100);

% 读取碰撞模型配置
jsonFilePath = "./model/collideConfig/S50_collision.json";
tooljsonFilePath = "./model/collideConfig/nonetool_collision.json";

fprintf('📂 加载S50碰撞模型配置...\n');
params = readCollisionModelJson(jsonFilePath);
params_tool = readToolCollisionJson(tooljsonFilePath);
fprintf('✅ 配置加载完成\n\n');

% 显示DH参数
fprintf('📐 S50 DH参数 (m): d1=%.4f, d2=%.4f, d3=%.4f, d4=%.4f, d5=%.4f, d6=%.4f, a2=%.4f, a3=%.4f\n\n', ...
        params.DH.d1, params.DH.d2, params.DH.d3, params.DH.d4, params.DH.d5, params.DH.d6, params.DH.a2, params.DH.a3);

%% ====================== 遍历所有姿态 ======================
numPoses = length(poses);
collisionResults = cell(numPoses, 1);

for poseIdx = 1:numPoses
    pose = poses{poseIdx};
    jointPositions = pose.joints;
    
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('🤖 姿态 %d/%d: %s\n', poseIdx, numPoses, pose.name);
    fprintf('   %s\n', pose.description);
    fprintf('   关节角 (deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', ...
        rad2deg(jointPositions(1)), rad2deg(jointPositions(2)), ...
        rad2deg(jointPositions(3)), rad2deg(jointPositions(4)), ...
        rad2deg(jointPositions(5)), rad2deg(jointPositions(6)));
    
    % 创建图形窗口
    fig = figure('Name', sprintf('码垛 - %s', pose.name), ...
           'Position', [50, 50, 1400, 1000], 'Color', 'white');
    hold on;
    
    % 设置全局透明度
    global alpha;
    alpha = 0.5;
    
    %% 绘制场景元素
    % 地面
    drawGroundPlane(-2, 2, -2, 2);
    
    % 电箱
    drawControlCabinet(cabinet);
    
    % 蓝色码垛框架
    drawBlueFrame(frame);
    
    % 传送带
    drawConveyorBelt(conveyor);
    
    % 箱子在传送带上（待抓取）
    boxPos1 = [conveyor.posX - 0.15, conveyor.posY - conveyor.width/4, ...
               conveyor.height + conveyor.beltHeight + box.height/2];
    drawCartonBox(boxPos1, box);
    
    % 如果是放置后的姿态，显示码垛框内已堆放的箱子
    if poseIdx >= 7
        % 第一层箱子位置
        stackedPos = [frame.posX + 0.15, frame.posY + frame.width/4, ...
                      frame.posZ + box.height/2 + 0.02];
        drawCartonBox(stackedPos, box);
    end
    
    %% 绘制机器人
    [outputStruct, T0T] = plotS50Robot(jointPositions, params, params_tool, cabinet.height);
    
    % 计算末端位置
    endPos = T0T(1:3, 4);
    endPos(3) = endPos(3) + cabinet.height;  % 加上平台高度
    
    % 检查末端方向
    % S-Serial机器人：末端坐标系Z轴是工具方向
    % 世界坐标系中，吸盘朝下意味着工具Z轴应该朝-Z方向
    endZ = T0T(1:3, 3);  % 末端Z轴在世界坐标系中的方向
    
    % 计算与垂直向下方向的夹角
    downDir = [0; 0; -1];
    dotProduct = dot(endZ, downDir);
    horizAngle = acosd(abs(dotProduct));  % 与垂直的偏差角度
    
    % 计算TCP到箱子顶部的距离（用于验证对齐精度）
    if poseIdx >= 2 && poseIdx <= 4  % 传送带侧
        targetBoxTop = [boxPos1(1), boxPos1(2), conveyor.height + conveyor.beltHeight + box.height];
        tcpToBoxDist = norm(endPos - targetBoxTop');
        fprintf('   末端位置 (m): [%.3f, %.3f, %.3f]\n', endPos(1), endPos(2), endPos(3));
        fprintf('   箱子顶部 (m): [%.3f, %.3f, %.3f]\n', targetBoxTop(1), targetBoxTop(2), targetBoxTop(3));
        fprintf('   TCP到箱距离: %.3fm\n', tcpToBoxDist);
    else
        fprintf('   末端位置 (m): [%.3f, %.3f, %.3f]\n', endPos(1), endPos(2), endPos(3));
    end
    
    fprintf('   末端Z轴方向: [%.2f, %.2f, %.2f]\n', endZ(1), endZ(2), endZ(3));
    if horizAngle < 15
        fprintf('   ✅ 末端垂直 (偏差 %.1f°)\n', horizAngle);
    elseif horizAngle < 30
        fprintf('   ⚠️  末端略倾斜 (偏差 %.1f°)\n', horizAngle);
    else
        fprintf('   ❌ 末端倾斜过大 (偏差 %.1f°)\n', horizAngle);
    end
    
    %% 碰撞检测
    [isCollision, collisionInfo, minDist] = checkCollision(outputStruct, params, frame, conveyor, cabinet);
    
    collisionResults{poseIdx} = struct('name', pose.name, ...
                                        'isCollision', isCollision, ...
                                        'info', collisionInfo, ...
                                        'minDist', minDist, ...
                                        'endPos', endPos);
    
    if isCollision
        fprintf('   ⚠️  碰撞: %s\n', collisionInfo);
        titleColor = [0.8 0 0];
    else
        fprintf('   ✅ 安全 (距离: %.3fm)\n', minDist);
        titleColor = [0 0.6 0];
    end
    
    %% 设置视图
    title(sprintf('HR S50-2000 码垛: %s', pose.name), ...
          'FontSize', 13, 'Color', titleColor, 'FontWeight', 'bold');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    axis equal; grid on;
    xlim([-2.0, 2.0]); ylim([-2.0, 2.0]); zlim([0, 3.0]);
    view(-50, 25);
    
    camlight('headlight'); lighting gouraud;
    drawnow;
    
    % 保存
    if isHeadless
        imgFile = sprintf('%s/pose_%02d.png', outputDir, poseIdx);
        saveas(fig, imgFile);
        close(fig);
    end
end

%% ====================== 结果汇总 ======================
fprintf('\n╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    码垛作业碰撞检测结果汇总                      ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════╣\n');

collisionCount = 0;
for i = 1:numPoses
    r = collisionResults{i};
    if r.isCollision
        status = '⚠️碰撞';
        collisionCount = collisionCount + 1;
    else
        status = '✅安全';
    end
    fprintf('║ %2d. %-12s %s (%.3fm) 末端:[%.2f,%.2f,%.2f] ║\n', ...
            i, r.name, status, r.minDist, r.endPos(1), r.endPos(2), r.endPos(3));
end

fprintf('╠══════════════════════════════════════════════════════════════════╣\n');
fprintf('║ 碰撞统计: %d/%d 姿态检测到碰撞                                   ║\n', collisionCount, numPoses);
fprintf('╚══════════════════════════════════════════════════════════════════╝\n');

if isHeadless
    fprintf('\n📁 图像保存至: %s\n', outputDir);
end

%% ==================== 辅助函数 ====================

function drawGroundPlane(xmin, xmax, ymin, ymax)
    v = [xmin ymin 0; xmax ymin 0; xmax ymax 0; xmin ymax 0];
    patch('Vertices', v, 'Faces', [1 2 3 4], ...
          'FaceColor', [0.9 0.9 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
end

function drawControlCabinet(cab)
    % 电箱 (机械臂基座下)
    x = -cab.width/2; y = -cab.depth/2; z = 0;
    v = [x y z; x+cab.width y z; x+cab.width y+cab.depth z; x y+cab.depth z;
         x y z+cab.height; x+cab.width y z+cab.height; 
         x+cab.width y+cab.depth z+cab.height; x y+cab.depth z+cab.height];
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', v, 'Faces', f, 'FaceColor', cab.color, ...
          'EdgeColor', [0.5 0.5 0.5], 'FaceAlpha', 0.95, 'LineWidth', 1);
    
    % 绿色显示屏
    screenV = [x+0.08 y+cab.depth-0.01 z+cab.height*0.4;
               x+cab.width-0.08 y+cab.depth-0.01 z+cab.height*0.4;
               x+cab.width-0.08 y+cab.depth-0.01 z+cab.height*0.7;
               x+0.08 y+cab.depth-0.01 z+cab.height*0.7];
    patch('Vertices', screenV, 'Faces', [1 2 3 4], 'FaceColor', [0.6 0.9 0.6], ...
          'EdgeColor', 'k', 'FaceAlpha', 1);
    
    % 红色急停按钮
    [sx, sy, sz] = sphere(16);
    btnR = 0.025;
    surf(sx*btnR + x + cab.width/2, sy*btnR + y + cab.depth - 0.01, ...
         sz*btnR + z + cab.height*0.25, 'FaceColor', [0.9 0.1 0.1], ...
         'EdgeColor', 'none');
end

function drawBlueFrame(frm)
    % 蓝色码垛框架
    r = frm.tubeRadius; w = frm.width; d = frm.depth; h = frm.height;
    cx = frm.posX; cy = frm.posY; cz = frm.posZ;
    
    corners = [cx-d/2 cy-w/2; cx+d/2 cy-w/2; cx+d/2 cy+w/2; cx-d/2 cy+w/2];
    
    % 立柱
    for i = 1:4
        drawTube(corners(i,1), corners(i,2), cz, corners(i,1), corners(i,2), cz+h, r, frm.color);
    end
    
    % 横梁
    heights = [0.05, h/3, 2*h/3, h-0.05];
    for hi = 1:length(heights)
        hz = cz + heights(hi);
        for i = 1:4
            j = mod(i, 4) + 1;
            drawTube(corners(i,1), corners(i,2), hz, corners(j,1), corners(j,2), hz, r*0.8, frm.color);
        end
    end
    
    % 网格
    for i = 1:4
        j = mod(i, 4) + 1;
        for k = 0:3
            zt = cz + k*h/4 + 0.1;
            drawTube(corners(i,1), corners(i,2), zt, ...
                     (corners(i,1)+corners(j,1))/2, (corners(i,2)+corners(j,2))/2, zt+h/8, ...
                     r*0.4, frm.color*0.95);
        end
    end
end

function drawTube(x1, y1, z1, x2, y2, z2, radius, color)
    [X, Y, Z] = cylinder(radius, 10);
    vec = [x2-x1; y2-y1; z2-z1];
    len = norm(vec);
    if len < 0.001, return; end
    Z = Z * len;
    
    % 旋转
    defDir = [0; 0; 1];
    tgtDir = vec / len;
    if norm(cross(defDir, tgtDir)) > 1e-6
        ax = cross(defDir, tgtDir); ax = ax/norm(ax);
        ang = acos(dot(defDir, tgtDir));
        R = axang2rot([ax', ang]);
    else
        R = eye(3);
        if dot(defDir, tgtDir) < 0, R(3,3) = -1; end
    end
    
    for i = 1:numel(X)
        pt = R * [X(i); Y(i); Z(i)];
        X(i) = pt(1) + x1; Y(i) = pt(2) + y1; Z(i) = pt(3) + z1;
    end
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.85);
end

function drawConveyorBelt(conv)
    x = conv.posX - conv.length/2; y = conv.posY - conv.width/2; z = 0;
    
    % 支架腿
    legW = 0.04;
    legs = [x+0.15 y+0.1; x+conv.length-0.15 y+0.1; 
            x+0.15 y+conv.width-0.1; x+conv.length-0.15 y+conv.width-0.1];
    for i = 1:4
        lv = [legs(i,1)-legW/2 legs(i,2)-legW/2 0;
              legs(i,1)+legW/2 legs(i,2)-legW/2 0;
              legs(i,1)+legW/2 legs(i,2)+legW/2 0;
              legs(i,1)-legW/2 legs(i,2)+legW/2 0;
              legs(i,1)-legW/2 legs(i,2)-legW/2 conv.height;
              legs(i,1)+legW/2 legs(i,2)-legW/2 conv.height;
              legs(i,1)+legW/2 legs(i,2)+legW/2 conv.height;
              legs(i,1)-legW/2 legs(i,2)+legW/2 conv.height];
        patch('Vertices', lv, 'Faces', [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
              'FaceColor', [0.25 0.25 0.25], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    end
    
    % 皮带
    bv = [x y conv.height; x+conv.length y conv.height;
          x+conv.length y+conv.width conv.height; x y+conv.width conv.height;
          x y conv.height+conv.beltHeight; x+conv.length y conv.height+conv.beltHeight;
          x+conv.length y+conv.width conv.height+conv.beltHeight; 
          x y+conv.width conv.height+conv.beltHeight];
    patch('Vertices', bv, 'Faces', [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
          'FaceColor', conv.color, 'EdgeColor', [0.2 0.2 0.2], 'FaceAlpha', 0.95);
    
    % 黄色条纹
    stripeY = y + conv.width/2;
    patch('Vertices', [x stripeY-0.02 conv.height+conv.beltHeight+0.001;
                       x+conv.length stripeY-0.02 conv.height+conv.beltHeight+0.001;
                       x+conv.length stripeY+0.02 conv.height+conv.beltHeight+0.001;
                       x stripeY+0.02 conv.height+conv.beltHeight+0.001], ...
          'Faces', [1 2 3 4], 'FaceColor', [0.9 0.8 0.2], 'EdgeColor', 'none');
end

function drawCartonBox(pos, bx)
    x = pos(1) - bx.length/2; y = pos(2) - bx.width/2; z = pos(3) - bx.height/2;
    v = [x y z; x+bx.length y z; x+bx.length y+bx.width z; x y+bx.width z;
         x y z+bx.height; x+bx.length y z+bx.height; 
         x+bx.length y+bx.width z+bx.height; x y+bx.width z+bx.height];
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', v, 'Faces', f, 'FaceColor', bx.color, ...
          'EdgeColor', [0.3 0.2 0.1], 'FaceAlpha', 0.98, 'LineWidth', 1.5);
    
    % 标签
    text(pos(1), pos(2), pos(3)+bx.height/2+0.01, 'Neurio', ...
         'FontSize', 7, 'HorizontalAlignment', 'center', 'Color', [0.2 0.1 0.05]);
end

function [outputStruct, T0T] = plotS50Robot(q, params, toolparams, baseHeight)
    global T6T d1 d2 d3 d4 d5 d6 a2 a3;
    
    d1 = params.DH.d1; d2 = params.DH.d2; d3 = params.DH.d3;
    d4 = params.DH.d4; d5 = params.DH.d5; d6 = params.DH.d6;
    a2 = -params.DH.a2; a3 = -params.DH.a3;
    T6T = eye(4);
    
    [T00, T01, T02, T03, T04, T05, T0T] = FK_SSerial(q);
    
    % 应用基座高度偏移到所有变换
    Tbase = eye(4); Tbase(3,4) = baseHeight;
    Tf_tree = {Tbase*T00, Tbase*T01, Tbase*T02, Tbase*T03, Tbase*T04, Tbase*T05, Tbase*T0T};
    
    % 绘制坐标系
    for i = 1:length(Tf_tree)
        plotframe(Tf_tree{i}, 0.08, true);
    end
    
    % 绘制碰撞模型
    outputStruct = plotSelfCollisonModel(Tf_tree, params, toolparams);
end

function [isCollision, info, minDist] = checkCollision(outputStruct, params, frm, conv, cab)
    isCollision = false;
    info = '';
    minDist = inf;
    
    % 获取碰撞体数据
    base_p1 = outputStruct.base_bc1(:);
    base_p2 = outputStruct.base_bc2(:);
    lowerArm_p1 = outputStruct.lowerArm_la1(:);
    lowerArm_p2 = outputStruct.lowerArm_la2(:);
    elbow_p1 = outputStruct.elbow_e1(:);
    elbow_p2 = outputStruct.elbow_e2(:);
    upperArm_p1 = outputStruct.upperArm_ua1(:);
    upperArm_p2 = outputStruct.upperArm_ua2(:);
    wrist_center = outputStruct.wrist_wc(:);
    
    r_base = params.base.radius;
    r_lowerArm = params.lowerArm.radius;
    r_elbow = params.elbow.radius;
    r_upperArm = params.upperArm.radius;
    r_wrist = params.wrist.radius;
    
    % 自碰撞检测对
    pairs = {
        {'基座-肘部', 'cap', [base_p1 base_p2], r_base, 'cap', [elbow_p1 elbow_p2], r_elbow};
        {'基座-上臂', 'cap', [base_p1 base_p2], r_base, 'cap', [upperArm_p1 upperArm_p2], r_upperArm};
        {'基座-腕部', 'cap', [base_p1 base_p2], r_base, 'sph', wrist_center, r_wrist};
        {'下臂-上臂', 'cap', [lowerArm_p1 lowerArm_p2], r_lowerArm, 'cap', [upperArm_p1 upperArm_p2], r_upperArm};
        {'下臂-腕部', 'cap', [lowerArm_p1 lowerArm_p2], r_lowerArm, 'sph', wrist_center, r_wrist};
    };
    
    for i = 1:length(pairs)
        p = pairs{i};
        dist = calcDist(p{2}, p{3}, p{5}, p{6});
        netDist = dist - p{4} - p{7};
        if netDist < minDist, minDist = netDist; end
        if netDist < 0
            isCollision = true;
            info = sprintf('自碰撞:%s', p{1});
            return;
        end
    end
    
    info = '安全';
end

function dist = calcDist(type1, data1, type2, data2)
    if strcmp(type1, 'cap') && strcmp(type2, 'cap')
        dist = segSegDist(data1(:,1), data1(:,2), data2(:,1), data2(:,2));
    elseif strcmp(type1, 'cap') && strcmp(type2, 'sph')
        dist = ptSegDist(data2, data1(:,1), data1(:,2));
    elseif strcmp(type1, 'sph') && strcmp(type2, 'cap')
        dist = ptSegDist(data1, data2(:,1), data2(:,2));
    else
        dist = norm(data1 - data2);
    end
end

function dist = ptSegDist(p, a, b)
    ab = b - a; ap = p - a;
    t = max(0, min(1, dot(ap, ab) / dot(ab, ab)));
    dist = norm(p - (a + t * ab));
end

function dist = segSegDist(p1, p2, p3, p4)
    d1 = p2 - p1; d2 = p4 - p3; r = p1 - p3;
    a = dot(d1,d1); b = dot(d1,d2); c = dot(d2,d2);
    dd = dot(d1,r); e = dot(d2,r);
    denom = a*c - b*b;
    if denom < 1e-10, s = 0; t = dd/max(b,1e-10);
    else, s = (b*e - c*dd)/denom; t = (a*e - b*dd)/denom; end
    s = max(0,min(1,s)); t = max(0,min(1,t));
    dist = norm((p1 + s*d1) - (p3 + t*d2));
end

function R = axang2rot(axang)
    ax = axang(1:3); ang = axang(4);
    c = cos(ang); s = sin(ang); t = 1 - c;
    x = ax(1); y = ax(2); z = ax(3);
    R = [t*x*x+c t*x*y-s*z t*x*z+s*y;
         t*x*y+s*z t*y*y+c t*y*z-s*x;
         t*x*z-s*y t*y*z+s*x t*z*z+c];
end
