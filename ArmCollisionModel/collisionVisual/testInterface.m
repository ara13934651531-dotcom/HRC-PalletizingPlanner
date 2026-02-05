clear; close all; clc;

addpath fileOperation;
addpath fileOperation/matlabjson/;
addpath plotGeometry;

global d1 d4 d6 a2;
global T6T;
% 设置DH参数
d1 = 0.220; d4 = 0.420; d6 = 0.185; a2 = 0.380;

% 从 JSON 文件读取参数
jsonFilePath ="../model/collideConfig/collsionParam2.json";
jsonSavePath = './data/output1.json';
params = readCollisionModelJson(jsonFilePath);

% 设置ToolInFlange
toolRadius = 0.1;
T6T = rpy2Rotation(0, 0, 0.1, 0, 0, 0);

% 设置当前关节角
q = [0  pi/3  pi/2  pi  pi/2  0]; % 工具和基座刚好碰撞
[T00, T01, T02, T03, T04, T05, T0T] = FK(q);
Tf_tree = {T00, T01, T02, T03, T04, T05, T0T};

outputStruct = plotSelfCollisonModel(Tf_tree, params);
saveUIdata(q, toolRadius, outputStruct, params, jsonSavePath);