close all;
clear all;
addpath collisionVisual\

addcollisionVisualPath;

%% 设置DH参数,设置机器人关节角度
robName = 'elfin3'; % 机器人类型 elfin3， elfin5， elfin10， elfin15
jointPositions = [0, pi/2, 0, 0, 0, 0]; % 机器人关节角度

%% 读取碰撞模型
% 从 JSON 文件读取参数
jsonFilePath ="./model/collideConfig/" + robName + "_collision.json";
tooljsonFilePath ="./model/collideConfig/nonetool_collision.json";
jsonSavePath = ['./data/', robName, '_output.json'];
params = readCollisionModelJson(jsonFilePath);
params_tool = readToolCollisionJson(tooljsonFilePath);

%% 进行碰撞几何仿真
% 创建机器人模型对象（以 elfin5 为例）
robotModel = RobotCollisionModel(robName);
robotModel.setJointPositions(jointPositions);

%% % 显示机器人模型
global alpha;
alpha = 0.2; % 画图的透明度
robotModel.showRobot(robName);  %showRobot是 RobotCollisiomModel中的一个类函数
outputStruct = plotCollisionModel(jointPositions, params, params_tool);
saveUIdata(jointPositions, outputStruct, params, jsonSavePath);

