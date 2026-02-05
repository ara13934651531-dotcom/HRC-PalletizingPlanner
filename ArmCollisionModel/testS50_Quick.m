% testS50_Quick.m - 快速版本（60帧，无机器人模型）
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

clear; clc; close all;
addpath('collisionVisual');
addpath(genpath('collisionVisual'));

fprintf('\n🚀 快速版码垛仿真 (仅场景+轨迹点)\n\n');

% DH参数 (米)
d1 = 0.2965; d2 = 0.3362; d3 = 0.2390;
d4 = 0.1585; d5 = 0.1585; d6 = 0.1345;
a2 = 0.9; a3 = 0.9415;

% 关键姿态 (度)
keyPoses = [
    0, -90, 30, 0, -60, 0;     % 初始
    60, -70, 50, 0, -70, 60;   % 传送带上方
    60, -50, 70, 0, -110, 60;  % 抓取
    60, -70, 50, 0, -70, 60;   % 抬起
    0, -80, 40, 0, -50, 0;     % 转向
    -60, -60, 60, 0, -90, -60; % 框内
    -60, -40, 80, 0, -130, -60;% 放置
    -60, -60, 60, 0, -90, -60; % 抬起
    -30, -70, 50, 0, -70, -30; % 退出
    0, -90, 30, 0, -60, 0;     % 返回
];

% 插值 (60帧总共)
nFrames = 60;
nKey = size(keyPoses, 1);
framesPerSeg = floor(nFrames / (nKey - 1));
trajectory = [];
for i = 1:nKey-1
    t = linspace(0, 1, framesPerSeg)';
    seg = (1-t) .* keyPoses(i,:) + t .* keyPoses(i+1,:);
    trajectory = [trajectory; seg(1:end-1,:)];
end
trajectory = [trajectory; keyPoses(end,:)];
trajectory = trajectory(1:nFrames, :);

% 简单FK计算TCP轨迹
tcpTrajectory = zeros(nFrames, 3);
for k = 1:nFrames
    q = deg2rad(trajectory(k,:));
    % 简化FK - 只计算TCP位置
    c1 = cos(q(1)); s1 = sin(q(1));
    c2 = cos(q(2)); s2 = sin(q(2));
    c23 = cos(q(2)+q(3)); s23 = sin(q(2)+q(3));
    c234 = cos(q(2)+q(3)+q(4)); s234 = sin(q(2)+q(3)+q(4));
    
    x = c1*(a2*c2 + a3*c23 + d5*s234);
    y = s1*(a2*c2 + a3*c23 + d5*s234);
    z = d1 + a2*s2 + a3*s23 - d5*c234 + d6;
    tcpTrajectory(k,:) = [x, y, z];
end

% 创建场景
figure('Position', [100 100 1000 800], 'Color', 'w');

% 绘制地板
[X, Y] = meshgrid(-1.5:0.5:2, -1:0.5:2);
Z = zeros(size(X));
surf(X, Y, Z, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
hold on;

% 绘制码垛框 (4根立柱)
frameCenter = [-0.5, 0.6];
frameW = 0.95/2; frameD = 0.75/2; frameH = 2.1;
pillarR = 0.025;
corners = [
    frameCenter(1)-frameW, frameCenter(2)-frameD;
    frameCenter(1)+frameW, frameCenter(2)-frameD;
    frameCenter(1)-frameW, frameCenter(2)+frameD;
    frameCenter(1)+frameW, frameCenter(2)+frameD;
];
[cyl_x, cyl_y, cyl_z] = cylinder(pillarR, 12);
cyl_z = cyl_z * frameH;
for i = 1:4
    surf(cyl_x + corners(i,1), cyl_y + corners(i,2), cyl_z, ...
        'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
end

% 绘制传送带
conveyorX = 0.85; conveyorZ = 0.85;
conveyorW = 0.42; conveyorL = 1.4;
patch([conveyorX-conveyorW/2, conveyorX+conveyorW/2, conveyorX+conveyorW/2, conveyorX-conveyorW/2], ...
      [-conveyorL/2, -conveyorL/2, conveyorL/2, conveyorL/2], ...
      [conveyorZ, conveyorZ, conveyorZ, conveyorZ], ...
      'FaceColor', [0.2 0.4 0.6], 'FaceAlpha', 0.7);

% 绘制箱子
boxW = 0.27; boxD = 0.33; boxH = 0.18;
boxYs = [-0.35, 0, 0.35];
for i = 1:3
    bx = conveyorX; by = boxYs(i); bz = conveyorZ;
    vertices = [
        bx-boxW/2, by-boxD/2, bz;
        bx+boxW/2, by-boxD/2, bz;
        bx+boxW/2, by+boxD/2, bz;
        bx-boxW/2, by+boxD/2, bz;
        bx-boxW/2, by-boxD/2, bz+boxH;
        bx+boxW/2, by-boxD/2, bz+boxH;
        bx+boxW/2, by+boxD/2, bz+boxH;
        bx-boxW/2, by+boxD/2, bz+boxH;
    ];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', [0.8 0.5 0.2], 'FaceAlpha', 0.8);
end

% 绘制机器人基座
[cyl_x, cyl_y, cyl_z] = cylinder(0.15, 20);
cyl_z = cyl_z * 0.1;
surf(cyl_x, cyl_y, cyl_z, 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');

% 绘制TCP轨迹
plot3(tcpTrajectory(:,1), tcpTrajectory(:,2), tcpTrajectory(:,3), 'r-', 'LineWidth', 2);

% 标记关键点
keyFrames = round(linspace(1, nFrames, nKey));
plot3(tcpTrajectory(keyFrames,1), tcpTrajectory(keyFrames,2), tcpTrajectory(keyFrames,3), ...
    'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% 当前TCP位置标记（初始位置）
hTCP = plot3(tcpTrajectory(1,1), tcpTrajectory(1,2), tcpTrajectory(1,3), ...
    'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

% 设置视角
view(45, 30);
axis equal;
xlim([-1.5 1.5]); ylim([-1 1.5]); zlim([0 2.5]);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('HR\_S50-2000 码垛仿真 - TCP轨迹预览', 'FontSize', 14);
grid on;
lighting gouraud;
camlight('headlight');

% 保存静态图
outputDir = 'pic/S50_Palletizing';
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

saveas(gcf, fullfile(outputDir, 'S50_trajectory_preview.png'));
fprintf('✅ 静态预览已保存: %s/S50_trajectory_preview.png\n', outputDir);

% 简单动画
fprintf('🎬 生成简化动画 (60帧)...\n');
gifFile = fullfile(outputDir, 'S50_quick_sim.gif');

for k = 1:nFrames
    % 更新TCP位置
    set(hTCP, 'XData', tcpTrajectory(k,1), 'YData', tcpTrajectory(k,2), 'ZData', tcpTrajectory(k,3));
    
    % 更新标题
    title(sprintf('HR\\_S50-2000 码垛仿真 - 帧 %d/%d', k, nFrames), 'FontSize', 14);
    
    drawnow;
    
    % 保存GIF
    frame = getframe(gcf);
    [imind, cm] = rgb2ind(frame.cdata, 256);
    if k == 1
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
    
    if mod(k, 20) == 0
        fprintf('   进度: %d/%d\n', k, nFrames);
    end
end

fprintf('✅ 快速动画已保存: %s\n', gifFile);
fprintf('\n🎉 完成!\n');
