clear; close all; clc;

addpath fileOperation;
addpath plotGeometry;
addpath kinematic\

global d1 d4 d6 a2;
global T6T;

% 从 JSON 文件读取参数
jsonFilePath ="../model/collideConfig/collsionParam2.json";
jsonSavePath = "./data/output1.json";
params = readCollisionModelJson(jsonFilePath);

% 设置DH参数
d1 = 0.220; d4 = 0.420; d6 = 0.185; a2 = 0.380;

% 设置ToolInFlange
T6T = rpy2Rotation(0, 0, 0.1, 0, 0, 0);

% 设置当前关节角
q = [0  pi/3  pi/2  pi  pi/2  0]; % 工具和基座刚好碰撞
[T00, T01, T02, T03, T04, T05, T0T] = FK(q);

% 绘图
figure
% bc = plotBall(T01, params.base.offset(1), params.base.offset(2), params.base.offset(3), params.base.radius); % Base
% hold on
% scatter3(bc(1), bc(2), bc(3), 'filled', 'r'); 
[bc1, bc2] = plotCapsule(T00, params.base.start(1), params.base.start(2), params.base.start(3), ...
    params.base.end(1), params.base.end(2), params.base.end(3), params.base.radius);

hold on
scatter3(bc1(1), bc1(2), bc1(3), 'filled', 'r'); 
scatter3(bc2(1), bc2(2), bc2(3), 'filled', 'r'); 

[la1, la2] = plotCapsule(T02, params.lowerArm.start(1), params.lowerArm.start(2), params.lowerArm.start(3), ...
    params.lowerArm.end(1), params.lowerArm.end(2), params.lowerArm.end(3), params.lowerArm.radius); % LowArm
scatter3(la1(1), la1(2), la1(3), 'filled', 'r'); 
scatter3(la2(1), la2(2), la2(3), 'filled', 'r'); 

hold on
[e1, e2] = plotCapsule(T03, params.elbow.start(1), params.elbow.start(2), params.elbow.start(3), ...
    params.elbow.end(1), params.elbow.end(2), params.elbow.end(3), params.elbow.radius); % Elbow
hold on

scatter3(e1(1), e1(2), e1(3), 'filled', 'r'); 
scatter3(e2(1), e2(2), e2(3), 'filled', 'r'); 

[ua1, ua2] = plotCapsule(T04, params.upperArm.start(1), params.upperArm.start(2), params.upperArm.start(3), ...
    params.upperArm.end(1), params.upperArm.end(2), params.upperArm.end(3), params.upperArm.radius); % UpperArm
hold on

scatter3(ua1(1), ua1(2), ua1(3), 'filled', 'r'); 
scatter3(ua2(1), ua2(2), ua2(3), 'filled', 'r'); 

wc = plotBall(T05, params.wrist.offset(1), params.wrist.offset(2), params.wrist.offset(3), params.wrist.radius); % Wrist
hold on
scatter3(wc(1), wc(2), wc(3), 'filled', 'r'); 
toolRadius = 0.06;
[tool1, tool2] = plotCapsule(T0T, 0, 0, -0.02, 0, 0, -0.16, toolRadius); % 工具

scatter3(tool1(1), tool1(2), tool1(3), 'filled', 'r'); 
scatter3(tool2(1), tool2(2), tool2(3), 'filled', 'r'); 

hold on
plotframe(T04, 0.1, true);
plotframe(T0T, 0.1, true);
plotframe(T00, 0.1, true);
% plotCube(-0.2, -0.3, -0.1, 1, 1, 0.2); % 工作台

offset = [0, 0, -0.25];
radius = 0.1;
drawBoxBallSweptVolume(eye(4), 1, 0.5, 0.3, radius, offset);
title("读取并验证空间包络模型")

paramOut.inputJoint = q*180/pi;
paramOut.base.start = bc1';
paramOut.base.end = bc2';
paramOut.base.radius = params.base.radius;
paramOut.lowerArm.start = la1';
paramOut.lowerArm.end = la2';
paramOut.lowerArm.radius = params.lowerArm.radius;
paramOut.elbow.start = e1';
paramOut.elbow.end = e2';
paramOut.elbow.radius = params.elbow.radius;
paramOut.upperArm.start = ua1';
paramOut.upperArm.end = ua2';
paramOut.upperArm.radius = params.upperArm.radius;
paramOut.wrist.offset = wc;
paramOut.wrist.radius = params.wrist.radius;
paramOut.tool.start = tool1';
paramOut.tool.end = tool2';
paramOut.tool.radius = toolRadius;

% json_data = jsonencode(paramOut);

% 将 JSON 数据写入文件
json_filename = './data/output1.json';
savejson('', paramOut, json_filename);

