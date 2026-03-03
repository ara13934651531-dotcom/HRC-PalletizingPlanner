function T_all = urdfFK(JOINTS, q_rad)
%URDFFK URDF正运动学: 返回8个变换矩阵
%   T{1}=base(eye4), T{2..7}=链路1..6, T{8}=TCP末端
%   elfin_end_joint: xyz=[0,0,0.1345] rpy=[0,0,pi] (固定关节, 无旋转自由度)
    T_all = cell(8,1); T_all{1} = eye(4);
    for i = 1:6
        xyz = JOINTS(i,1:3); rpy = JOINTS(i,4:6);
        T_all{i+1} = T_all{i} * makeTrans(xyz) * makeRotRPY(rpy) * makeRotZ(q_rad(i));
    end
    % 添加末端关节: elfin_end_joint (固定偏移 134.5mm + 绕Z旋转180°)
    T_all{8} = T_all{7} * makeTrans([0, 0, 0.1345]) * makeRotRPY([0, 0, pi]);
end
