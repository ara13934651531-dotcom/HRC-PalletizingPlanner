close all;
clear all;
addpath collisionVisual/

addcollisionVisualPath;

%% 设置DH参数,设置机器人关节角度
robName = 'S05'; % 机器人类型 S05， S10， S20， S30， S35
% jointPositions = [pi/2, -pi/2 , -pi/2, -pi/4, pi/2, 0]; % 机器人关节角度
% jointPositions = [0, pi/4, -pi*3/4, -3*pi/4, pi/2, 0]; % 机器人关节角度
% jointPositions = [0, pi/4, -pi*2.8/4, -3*pi/4, pi/2, 0]; % 机器人关节角度
% jointPositions = [0, pi/4, -pi*3/4, -3*pi/4, pi/2, 3*pi/4]; % 机器人关节角度

% jointPositions = [8.766,-149.808,112.881,-89.994,-88.813,0.003]/180*pi;

jointPositions =  [0.000,-90.000,-141.936,-89.204,90.000,0.000]/180*pi;

%% 读取碰撞模型
% 从 JSON 文件读取参数
jsonFilePath ="./model/collideConfig/" + robName + "_collision.json";
tooljsonFilePath ="./model/collideConfig/S_tool_collision_Burongbin.json";
jsonSavePath = ['./data/', robName, '_output.json'];
params = readCollisionModelJson(jsonFilePath);
params_tool = readToolCollisionJson(tooljsonFilePath);

%% 进行碰撞几何仿真
% 创建机器人模型对象（以 elfin5 为例）
robotModel=RobotCollisionModel(robName);
robotModel.setJointPositions(jointPositions);

%% % 显示机器人模型
global alpha;
alpha = 0.2; % 画图的透明度
robotModel.showRobot(robName);
outputStruct = plotSSerialCollisionModel(jointPositions, params, params_tool);
saveUIdata(jointPositions, outputStruct, params, jsonSavePath);
title(robName);
view(200,30);
