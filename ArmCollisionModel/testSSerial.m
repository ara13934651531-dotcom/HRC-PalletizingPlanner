close all;
clear all;
addpath collisionVisual\

addcollisionVisualPath;

%% 设置DH参数,设置机器人关节角度
robName = 'S20'; % 机器人类型 S05， S10， S20， S30， S35
% jointPositions = [pi/2, -pi/2 , -pi/2, -pi/4, pi/2, 0]; % 机器人关节角度
jointPositions = [0, pi/4, -pi*3.5/4, pi, 0, 0]; % 机器人关节角度

%% 读取碰撞模型
% 从 JSON 文件读取参数
jsonFilePath ="./model/collideConfig/" + robName + "_collision.json";
tooljsonFilePath ="./model/collideConfig/nonetool_collision.json";
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
