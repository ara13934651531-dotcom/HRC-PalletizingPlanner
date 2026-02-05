classdef RobotCollisionModel < handle
    properties
        robot;  % 机器人对象
        stlPath;  % 模型路径
        urdfPath;
        robotConfig;
    end
    
    methods
        % 构造函数
        function obj = RobotCollisionModel(robotType)
            % 根据机器人类型设置模型路径和导入机器人
            obj.urdfPath = ['model/urdf/', robotType,'.urdf'];
            obj.stlPath = ['model/meshes/', robotType];
            % switch robotType
            %     case 'elfin5'
            %         obj.urdfPath = ['model\urdf\', robotType,'.urdf'];
            %         obj.stlPath = ['model\meshes\', robotType];
            %     otherwise
            %         error('Unsupported robot type');
            % end
            
            obj.robot = importrobot(obj.urdfPath, 'MeshPath', obj.stlPath);
            % Iterate through all links of the robot
            % obj.robot.
        end
        
        function addTool(obj,toolPath)
            % "model\meshes\tool\Paint_gun4.stl"
            obj.robot.Bodies{10}.addVisual("Mesh",toolPath);
        end


        
        % 显示机器人模型
        function showRobot(obj, dual_arm)
            show(obj.robot, obj.robotConfig);  %在图形窗口中显示机器人模型（obj.robot），并按照 obj.robotConfig 中存储的关节配置来展示机器人的姿态
            axis equal;  % 使坐标轴比例相等
            xlim([-1, 1]);  % 设置X轴范围
            ylim([-1, 1]);  % 设置Y轴范围
            zlim([-0.1, 1]);  % 设置Z轴范围（如果有必要）
        end
        

        
       %设置机器人关节姿态
        function setJointPositions(obj, jointPositions)
            if length(jointPositions) ~= 6
                error('Invalid number of joint positions');
            end

            obj.robotConfig = homeConfiguration(obj.robot); %homeConfiguration 函数的唯一入参是机器人模型对象，本质是一个 "安全起点" 函数，为机器人提供一套默认、合法的关节角度配置，确保后续姿态调整有可靠的基础。
            for i = 1:length(jointPositions)
                obj.robotConfig(i).JointPosition = jointPositions(i);
            end
        end
    end
end
