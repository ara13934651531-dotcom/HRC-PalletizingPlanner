clc;
clear;

% 定义URDF文件路径（根据你的实际路径调整）
robotType = 'elfin5';  % 假设机器人类型为'elfin5'
urdfPath = ['model/urdf/', robotType, '.urdf'];
stlPath = ['model/meshes/', robotType];

% 导入URDF模型
robot = importrobot(urdfPath, 'MeshPath', stlPath);
robot.DataFormat = 'row';  % 设置数据格式为行向量
robot.Gravity = [0, 0, -9.81];  % 设置重力

% 定义关节状态：关节角度、速度、加速度
jointAngles = [0, pi/6, pi/4, pi/3, pi/2, 0];  % 关节角度 (假设5个关节)
jointVelocities = zeros(size(jointAngles));  % 初始关节速度为0
jointAccelerations = zeros(size(jointAngles));  % 初始关节加速度为0

% 计算逆动力学以获取关节力矩
torques = inverseDynamics(robot, jointAngles, jointVelocities, jointAccelerations);

% 显示结果
disp('关节角度:');
disp(jointAngles);
disp('计算的关节力矩:');
disp(torques);
