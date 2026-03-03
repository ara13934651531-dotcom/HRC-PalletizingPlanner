function [joints_m, bodies] = soFK2Skeleton(q_deg)
%SOFK2SKELETON 使用SO库获取真实DH碰撞体骨架 (v1.0.0)
%   调用 getUIInfoMationInterface 获取碰撞体全局位置 (mm)
%   调用 forwardKinematics2 获取TCP (mm)
%   返回:
%     joints_m: 5x3 骨架关键点 (m, 机器人基座坐标系)
%               [BaseBottom; BaseTop; Elbow; Wrist; TCP]
%     bodies: 1x5 struct array, 碰撞体几何
%             .type: 'capsule'|'ball'|'none'
%             .p1/.p2: 端点 (m, 基座坐标系)
%             .center: 中心 (m)
%             .radius_m: 碰撞半径 (m)
    
    % update() 设置SO内部状态 (getUIInfo依赖此状态)
    vel = zeros(1,6); acc = zeros(1,6);
    calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', q_deg, vel, acc);
    
    % 获取碰撞体全局位置 (mm, FK2坐标系)
    collIdx = int32(zeros(1,7)); collType = int32(zeros(1,7));
    dataList = zeros(1,63); radiusList = zeros(1,7);
    [~, collType, dataList, radiusList] = calllib('libHRCInterface', ...
        'getUIInfoMationInterface', collIdx, collType, dataList, radiusList);
    dataM = reshape(dataList, 9, 7)';  % 7x9 matrix
    
    % FK2 获取TCP (mm)
    tcpS = libstruct('MC_COORD_REF');
    tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
    [~, ~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);
    tcp_m = [tcpS.X, tcpS.Y, tcpS.Z] / 1000;  % mm → m
    
    % 提取5个碰撞体几何 (Base/LowerArm/Elbow/UpperArm/Wrist)
    bodies = struct('type', cell(1,5), 'p1', cell(1,5), 'p2', cell(1,5), ...
                    'center', cell(1,5), 'radius_m', cell(1,5));
    for bi = 1:5
        d = dataM(bi,:);
        bodies(bi).radius_m = radiusList(bi) / 1000;
        if collType(bi) == 2  % Capsule
            bodies(bi).type = 'capsule';
            bodies(bi).p1 = [d(1), d(2), d(3)] / 1000;
            bodies(bi).p2 = [d(4), d(5), d(6)] / 1000;
            bodies(bi).center = (bodies(bi).p1 + bodies(bi).p2) / 2;
        elseif collType(bi) == 1  % Ball
            bodies(bi).type = 'ball';
            bodies(bi).center = [d(1), d(2), d(3)] / 1000;
            bodies(bi).p1 = bodies(bi).center;
            bodies(bi).p2 = bodies(bi).center;
        else
            bodies(bi).type = 'none';
            bodies(bi).p1 = [0 0 0]; bodies(bi).p2 = [0 0 0];
            bodies(bi).center = [0 0 0];
        end
    end
    
    % 骨架关键点: 从碰撞体端点提取
    %   BaseBottom = body1.p1, BaseTop = body1.p2
    %   Elbow = body2.p2 (大臂远端), Wrist = body3.p2 (小臂远端)
    %   TCP = FK2
    joints_m = [
        bodies(1).p1;       % 基座底部
        bodies(1).p2;       % 基座顶部 / 肩
        bodies(2).p2;       % 大臂远端 / 肘
        bodies(3).p2;       % 小臂远端 / 腕
        tcp_m;              % TCP (FK2)
    ];
end
