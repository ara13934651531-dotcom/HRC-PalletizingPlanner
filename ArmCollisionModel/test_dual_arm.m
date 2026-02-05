close all;
clear all;
addpath collisionVisual/

addcollisionVisualPath;

%% 设置DH参数,设置机器人关节角度
robName = 'dual_arm'; 

jointPositions0 = [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0]*pi/180; % 机器人关节角度

% 碰撞1
jointPositions1 = [0, -90, 35, 0, 0, 0,    0, -90, 35, 0, 0, 0]*pi/180; % 机器人关节角度

% 碰撞2
jointPositions2 = [0, -45, 135, 0, 120, 0,    0, 0, 0, 0, 0, 0]*pi/180; % 机器人关节角度

% 碰撞3
jointPositions3 = [0, 0, 0, 0, 0, 0,    0, -10, 140, 0, 140, 0]*pi/180; % 机器人关节角度

% 碰撞4
jointPositions4 = [0, -80, 100, 0, 0, 0,    0, -120, 0, 0, 0, 0]*pi/180; % 机器人关节角度

% 碰撞5
jointPositions5 = [0, -100, 30, 45, 0, 0,    10, -70, 90, 0, 0, 0]*pi/180; % 机器人关节角度

% 碰撞6
jointPositions6 = [0, 0, 0, 0, 0, 0,    -90, -20, 100, 0, 50, 0]*pi/180; % 机器人关节角度

% 碰撞7
jointPositions7 = [0, 0, 0, 0, 0, 0,    -90, 50, 140, 0, 0, 0]*pi/180; % 机器人关节角度

%% 读取碰撞模型
% 从 JSON 文件读取参数
jsonFilePath ="./model/collideConfig/" + robName + "_collision.json";
tooljsonFilePath ="./model/collideConfig/dualarm_tool_collision.json";
jsonSavePath = ['./data/', robName, '_output.json'];
params = read_dual_arm_ModelJson(jsonFilePath);
toolparams = readToolCollisionJson(tooljsonFilePath);


%% 进行碰撞几何仿真
jointPositions = jointPositions0;

robotModel = dual_armRobotCollisionModel(robName);
robotModel.setJointPositions(jointPositions);
% robotModel.addTool(toolurdfPath);

%% % 显示机器人模型q
global alpha;
alpha = 0.2; % 画图的透明度
robotModel.showRobot(robName);  %showRobot是 RobotCollisiomModel中的一个类函数
outputStruct = plot_dual_armCollisionModel(jointPositions, params, toolparams); hold on;
% dual_arm_saveUIdata(jointPositions, outputStruct, params, jsonSavePath);
