function joints_m = fk2Skeleton(q_deg)
%FK2SKELETON 旧压缩FK2骨架模型 (DEPRECATED)
%   ⚠ v1.0.0后尺度不正确! 仅在SO库未加载时作为回退使用.
%   正常情况下使用 soFK2Skeleton().
%   旧模型总臂长~1.175m, 真实DH总臂长~1.842m (差异~57%)
    H = 0.220; L1 = 0.380; L2 = 0.420; L3 = 0.155;
    q1 = q_deg(1); q2 = q_deg(2); q3 = q_deg(3); q5 = q_deg(5);
    arm_dir = [cosd(q1), sind(q1), 0];
    a2 = -q2; a23 = -q2 + q3; a235 = -q2 + q3 + q5;
    base     = [0, 0, 0];
    shoulder = [0, 0, H];
    elbow    = shoulder + L1 * [sind(a2)*arm_dir(1), sind(a2)*arm_dir(2), cosd(a2)];
    wrist    = elbow    + L2 * [sind(a23)*arm_dir(1), sind(a23)*arm_dir(2), cosd(a23)];
    tcp_pos  = wrist    + L3 * [sind(a235)*arm_dir(1), sind(a235)*arm_dir(2), cosd(a235)];
    joints_m = [base; shoulder; elbow; wrist; tcp_pos];
end
