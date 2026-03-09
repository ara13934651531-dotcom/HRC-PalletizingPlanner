% helpers/ — testS50_Palletizing_v15 (v18.0) 辅助函数目录
%
% 活跃函数: 32个 (已归档6个不再使用的函数到 archive/)
% 主脚本通过 addpath(fullfile(fileparts(mfilename('fullpath')), 'helpers'))
% 自动将此目录添加到MATLAB路径。
%
% ═══════════════════════ FK / 运动学 ═══════════════════════
%   urdfFK.m              - URDF正运动学 (8个变换矩阵, STL渲染用)
%   fk2Skeleton.m         - [DEPRECATED] 旧压缩FK2骨架 (仅无SO回退)
%   soFK2Skeleton.m       - SO库真实DH碰撞体骨架 (v1.0.0, 主要FK)
%
% ═══════════════════ 碰撞分析 (v6.3) ══════════════════════
%   boxArmCollDist.m       - ★ 独立箱子-机械臂碰撞距离分析
%                              26点OBB采样, Base+LowArm硬约束
%   boxOBBClearance.m      - ★ OBB间隙分析 (v18.0新增)
%   optimizeBoxRotation.m  - ★ J6旋转优化 (v18.0新增, TCP旋转避障)
%
% ═══════════════════ 机器人渲染 ═══════════════════════════
%   renderSTLRobot.m           - STL网格渲染 (静态单帧)
%   renderCapsuleRobotHandles.m - 碰撞包络渲染 (SO真实碰撞体, 动画用)
%
% ══════════════════ 绘图原语 ═══════════════════════════════
%   drawCapsule3D_h.m      - 胶囊体 (返回hggroup句柄, 可删除)
%   drawCylinder3D.m       - 圆柱体 (环境碰撞体可视化)
%
% ═══════════════════ 场景元素 ═══════════════════════════════
%   drawGround_v11.m       - 地面 (网格+机器人标注)
%   drawCabinet_v11.m      - 电箱 (4面+顶面)
%   drawFrame_v11.m        - 框架 (4立柱+横梁+管结构)
%   drawPallet_v11.m       - 托盘 (木板条+标注)
%   drawConveyor_v15.m     - 传送带 (滚筒+支腿)
%   drawBox3D_v11.m        - 实心六面体 (被drawConveyor_v15调用)
%   drawBox_v11.m          - 箱子居中定位 (主脚本直接调用)
%   drawRotatedBox.m       - 旋转箱子 (v18.0新增, TCP旋转避障动画)
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
% ══════════════════ 已归档 (archive/) ═════════════════════
%   drawCapsule3D.m        - 被drawCapsule3D_h替代 (无句柄返回)
%   renderSTLRobotOnBase.m - 未使用 (含基座偏移版本)
%   renderSTLRobotHandles.m - 未使用 (返回句柄版本)
%   drawOBBWall3D.m        - 未使用 (Lozenge OBB面板)
%   drawTube_v11.m         - 未使用 (管子/横梁)
%   drawWallPanel3D.m      - 未使用 (墙面板)
%
% @date   2026-03-05
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
