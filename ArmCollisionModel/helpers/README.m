% helpers/ — testS50_Palletizing_v15 辅助函数目录
%
% 从主脚本 testS50_Palletizing_v15.m 提取的33个独立函数文件。
% 主脚本通过 addpath(fullfile(fileparts(mfilename('fullpath')), 'helpers'))
% 自动将此目录添加到MATLAB路径。
%
% ═══════════════════════ FK / 运动学 ═══════════════════════
%   urdfFK.m              - URDF正运动学 (8个变换矩阵)
%   fk2Skeleton.m         - [DEPRECATED] 旧压缩FK2骨架模型
%   soFK2Skeleton.m       - SO库真实DH碰撞体骨架 (v1.0.0)
%
% ═══════════════════ 机器人渲染 ═══════════════════════════
%   renderSTLRobot.m           - STL网格渲染
%   renderSTLRobotOnBase.m     - STL网格渲染 (含基座偏移)
%   renderSTLRobotHandles.m    - STL网格渲染 (返回句柄)
%   renderCapsuleRobotHandles.m - 碰撞包络渲染 (SO真实碰撞体)
%
% ══════════════════ 绘图原语 ═══════════════════════════════
%   drawCapsule3D.m        - 胶囊体 (圆柱+两端球)
%   drawCapsule3D_h.m      - 胶囊体 (返回hggroup句柄)
%   drawCylinder3D.m       - 圆柱体 (无端盖)
%   drawWallPanel3D.m      - 墙面板 (扁平矩形)
%   drawOBBWall3D.m        - Lozenge OBB墙面板
%   drawTube_v11.m         - 管子/横梁
%
% ═══════════════════ 场景元素 ═══════════════════════════════
%   drawGround_v11.m       - 地面
%   drawCabinet_v11.m      - 电箱
%   drawFrame_v11.m        - 框架 (4立柱+横梁)
%   drawPallet_v11.m       - 托盘
%   drawConveyor_v15.m     - 传送带
%   drawBox3D_v11.m        - 实心六面体
%   drawBox_v11.m          - 箱子 (居中定位)
%
% ══════════════════ 信息面板 ═══════════════════════════════
%   drawInfoPanel_v15.m    - 实时信息面板 (环境碰撞+搬运状态)
%
% ══════════════════ 数据 I/O ═══════════════════════════════
%   loadNumericData.m      - 加载空格分隔数值文件
%   readSummaryFile.m      - 解析 key:value summary文件
%   saveFig.m              - 保存 PNG + .fig
%   saveKeyframeFig.m      - 保存动画关键帧
%
% ══════════════════ 数学工具 ═══════════════════════════════
%   makeTrans.m            - 4x4 平移矩阵
%   makeRotX.m / Y / Z    - 绕轴旋转矩阵
%   makeRotRPY.m           - URDF RPY旋转 (Rz*Ry*Rx)
%   axang2r_local.m        - 轴角→旋转矩阵
%
% ══════════════════ 通用工具 ═══════════════════════════════
%   ifelse.m               - 条件选择 (三元运算符)
%   getField.m             - 安全获取结构体字段
%
% @date   2026-02-24
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
