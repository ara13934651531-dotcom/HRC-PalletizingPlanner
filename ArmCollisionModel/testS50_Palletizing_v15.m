function testS50_Palletizing_v15()
%% testS50_Palletizing_v15 - HR_S50-2000 v5.0 正确布局 + 完整碰撞 + 3D仿真
%  v15.5 — 基于 v15.4 + 真实DH碰撞体渲染:
%
%  v15.5 修复清单:
%    1. renderCapsuleRobotHandles 改用 SO 库 getUIInfoMation 真实碰撞体
%       - 替代旧 fk2Skeleton 经验骨架模型 (坐标系完全错误)
%       - 碰撞体位置直接来自 SO 库 (mm, DH物理坐标系)
%       - FK2 获取精确 TCP 位置 (mm, 真实DH坐标)
%    2. TCP 轨迹计算改用 soFK2_tcp (真实DH FK2)
%       - Fig 4/7 TCP 轨迹: fk2Skeleton → soFK2_tcp
%       - 保留 fk2Skeleton 作为 SO 不可用时的后备
%    3. s50_collision_matlab.h 修复:
%       - FK2 声明为 void (匹配 MATLAB calllib 输出参数顺序)
%       - 注释掉 MC_COORD_REF by-value 返回函数 (MATLAB 不支持)
%       - FK2 文档: m → mm (v1.0.0)
%    4. 新增辅助函数: soFK2_tcp, soGetCollisionBodies
%
%  v15.4 修复清单 (保留):
%    1. SO库升级至 libHRCInterface v1.0.0
%       - 新初始化流程: initilizeRobotType(1) → setKinParams → initAC
%       - SO_INT 从 short 改为 int (int16→int32)
%    2. FK2 输出单位从 m 改为 mm (v1.0.0 返回真实DH物理坐标, mm)
%       - scanLastTcp_mm 移除 *1000 (FK2已返回mm)
%       - pall_tcp_so 回退分支移除 /1000
%    3. FK2 v1.0.0 返回真实 DH 物理坐标系 (非旧版压缩FK2)
%       - ZERO [0,0,0,0,0,0] → TCP=(-1841.5, -390.2, 138.0) mm
%       - HOME [0,-90,0,0,90,0] → TCP=(-158.5, -255.7, 2272.5) mm
%    4. [已完成 v15.5] fk2Skeleton() 已替换为 SO getUIInfoMation 真实碰撞体
%       - 当前仅影响3D渲染 (机器人骨架位置近似), 不影响数据正确性
%       - 所有碰撞/IK/FK数据通过SO库获取, 坐标系正确
%
%  v15.3 修复清单 (保留):
%    1. 所有渲染统一使用FK2骨架模型 (与场景坐标系一致)
%    2. cfg_nBoxes默认1 (逐步验证, 确保正确后再扩展)
%    3. C++同步限制为1个箱子 (MAX_BOXES=1)
%
%  v15.2 修复清单 (保留):
%    2. 框架顶梁改为平行于Y轴 (匹配实物: 连接近端→远端立柱)
%    3. 移除框架侧挡板碰撞体 (实物框架无中层横杆)
%    4. 近端面仅保留底边 (无顶边/中间栏杆, 开放式便于机器人进出)
%
%  v15.1 修复清单:
%    1. SO库环境碰撞注册: 框架4柱+2顶梁, 电箱4边, 传送带3面
%    2. 动画循环工具碰撞球管理: seg2启用/seg6移除 + 已放置箱子注册
%    3. TCP姿态分析使用FK2 A,B,C (ZYX Euler), 消除urdfFK混用 (#4指导规则)
%    4. 碰撞包络半径匹配CollisionGeometry.hpp (160/140/120/100mm)
%    5. drawFrame_v11渲染: 近端底边+远端底顶+左右顶梁 (匹配实物框架)
%    6. TCP方向quiver使用FK2导出Z轴, 非骨架近似
%    7. cfg_nBoxes可调, TCP_ORIENT_TOL_DEG=30°, cfg_convGap=0.20 匹配C++
%    8. 框架碰撞半径 30mm→50mm (匹配C++ FRM_TUBE_R+20)
%    9. pall_tcp_so 单位修复 (mm/1000→m, 非*1000)
%
%  v15.0 修复清单:
%    1. 修复makeRotRPY: URDF标准 Rz*Ry*Rx (之前是Rx*Ry*Rz → 机械臂渲染错误)
%    2. 场景布局匹配C++ v5.0: FRM_D=650,FRAME_GAP=50,PAL_H=500,CONV_GAP=200
%    3. 箱子放置位读取C++实际IK求解TCP位置 (消除MATLAB独立计算的偏差)
%    4. 完整环境碰撞体: 电箱(4)+传送带(3)+框架(4)+已放箱(12) 全可视化
%    5. 码垛顺序: 里→外,左→右,下→上 与C++ FIFO匹配
%
%  数据源:
%    data/so_palletizing_trajectory.txt — C++码垛轨迹 (19列, deg)
%    data/so_palletizing_profile.csv    — 碰撞距离曲线 (16列)
%    data/so_palletizing_summary.txt    — v5.0摘要 (含IK TCP位置)
%    data/so_collision_trajectory.txt   — C++碰撞仿真轨迹 (27列, deg)
%    libHRCInterface.so — 实时碰撞检测 (HansAlgorithmExport)
%
%  @file   testS50_Palletizing_v15.m
%  @brief  HR_S50-2000 v5.0 精确布局 + 完整环境碰撞可视化
%  @date   2026-02-24
%  Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

close all; clc;
tTotal = tic;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                    用户可配置参数区                                  ║
%% ╚══════════════════════════════════════════════════════════════════════╝

% --- 碰撞.so库路径 (优先使用环境变量 HRC_LIB_PATH) ---
SO_PATH = getenv('HRC_LIB_PATH');
if isempty(SO_PATH)
    % 回退: 尝试项目 lib/ 目录
    SO_PATH = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'lib', 'libHRCInterface.so');
    if ~isfile(SO_PATH)
        error('libHRCInterface.so 未找到, 请设置环境变量 HRC_LIB_PATH 或将其放入 lib/ 目录');
    end
end
SO_HEADER = fullfile(fileparts(mfilename('fullpath')), 's50_collision_matlab.h');

% --- S50 DH参数 (mm) ---
DH_MM = [296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5];

% --- S50 碰撞几何 (mm, 局部坐标系) ---
COLLISION_GEO.base      = [0,0,20, 0,0,330, 160];
COLLISION_GEO.lowerArm  = [0,0,340, 900,0,340, 140];
COLLISION_GEO.elbow     = [-10,0,60, 941.5,0,60, 120];
COLLISION_GEO.upperArm  = [0,0,-50, 0,0,100, 100];
COLLISION_GEO.wrist     = [0,0,20, 140];

% --- URDF 关节参数 [x y z roll pitch yaw] ---
JOINTS = [
    0,       0, 0.2833,        0,      0,  pi/2;
   -0.3345,  0, 0,          pi/2,      0, -pi/2;
   -0.9,     0, -0.239,        0,      0,  pi;
    0.9415,  0, 0,              0,      0,  0;
    0,       0, 0.1585,    -pi/2,      0,  0;
    0,       0, 0.1585,     pi/2,      0,  0;
];

% --- 场景参数 ---
% --- 场景参数 (匹配C++ v5.0 scene namespace) ---
cfg_cab.widthX=0.55; cfg_cab.depthY=0.65; cfg_cab.heightZ=0.80;
cfg_cab.color=[0.95,0.95,0.93];
cfg_frame.widthX=1.20; cfg_frame.depthY=0.65; cfg_frame.height=2.00;  % 深度650mm匹配C++
cfg_frame.tubeR=0.030; cfg_frame.color=[0.25,0.55,0.85];
cfg_pallet.widthX=1.00; cfg_pallet.depthY=0.60; cfg_pallet.heightZ=0.50;  % PAL_H=500mm
cfg_pallet.color=[0.20,0.45,0.80];
cfg_conv.lengthY=3.50; cfg_conv.widthX=0.55; cfg_conv.heightZ=0.75;
cfg_conv.beltH=0.035; cfg_conv.rollerR=0.030; cfg_conv.nRollers=18;
cfg_conv.color=[0.30,0.30,0.32];
cfg_box.lx=0.35; cfg_box.wy=0.28; cfg_box.hz=0.25;
cfg_box.color=[0.65,0.45,0.25];
cfg_nBoxes = 1;   % 箱子数目 (可调: 1=验证, 3=演示, 12=完整码垛)
cfg_animTaskLimit = 0;  % 0=全部任务 (调试时可设为3限制前N个)
cfg_frameGap = 0.05; cfg_convGap = 0.20;  % 匹配C++ CONV_GAP=200mm
cfg_convOffY = -0.80; cfg_convBoxYStart = -1.50; cfg_convBoxYStep = 0.25;  % 箱子从Y=-2.30开始,12个均在传送带范围内

% --- 环境碰撞体 (由场景参数动态计算, 与C++ v5.0一致) ---
% 注: 坐标为机器人基座坐标系(m), 下方初始化时根据scene参数计算
ENV_COLL.boxR_m = 0.250;   % 已放置箱碰撞球半径
ENV_COLL.toolR_m = 0.225;  % 工具碰撞球半径
ENV_COLL.cabR_m = 0.080;   % 电箱碰撞胶囊半径
ENV_COLL.convR_m = 0.080;  % 传送带碰撞胶囊半径

% --- 渲染参数 ---
CYL_N = 6;
VIEW_AZ = 135; VIEW_EL = 25;
MESH_REDUCE_RATIO = 0.3;

LINK_COLORS = {
    [0.85, 0.85, 0.88];  [0.15, 0.15, 0.18];  [0.92, 0.92, 0.94];
    [0.92, 0.92, 0.94];  [0.15, 0.15, 0.18];  [0.92, 0.92, 0.94];
    [0.15, 0.15, 0.18];
};
LINK_ALPHA = 0.92;

% --- 动画 ---
ANIM_SUBSAMPLE = 10;
GIF_SUBSAMPLE  = 30;

% --- TCP姿态约束 ---
TCP_DESIRED_AXIS = [0; 0; -1];
TCP_ORIENT_TOL_DEG = 30.0;  % 匹配C++ orientTolerance_deg = 30.0

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                      系统初始化                                     ║
%% ╚══════════════════════════════════════════════════════════════════════╝

CJK_FONT = 'Noto Sans CJK SC';
allFonts = listfonts();
if ~any(strcmp(allFonts, CJK_FONT))
    candidates = {'Noto Serif CJK SC','SimHei','WenQuanYi Micro Hei','Arial Unicode MS'};
    CJK_FONT = '';
    for ci = 1:length(candidates)
        if any(strcmp(allFonts, candidates{ci})), CJK_FONT = candidates{ci}; break; end
    end
    if isempty(CJK_FONT), CJK_FONT = get(0, 'DefaultAxesFontName'); end
end
set(0, 'DefaultAxesFontName', CJK_FONT);
set(0, 'DefaultTextFontName', CJK_FONT);

isHeadless = ~usejava('desktop');
outputDir = './pic/S50_palletizing_v15';
if isHeadless, set(0, 'DefaultFigureVisible', 'off'); end
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

projRoot = fileparts(fileparts(mfilename('fullpath')));
meshDir  = fullfile(projRoot, 'ArmCollisionModel', 'model', 'meshes', 'S50');
if ~exist(meshDir, 'dir')
    meshDir = fullfile(projRoot, 'S50_ros2', 'S50_meshes_dir', 'S50');
end
dataDir  = fullfile(projRoot, 'data');

% 场景计算
cab=cfg_cab; frame=cfg_frame; pallet=cfg_pallet; conv=cfg_conv; box=cfg_box;
nBoxes=cfg_nBoxes;
baseX=0; baseY=0; baseZ=cab.heightZ;
frame.cx = 0.0;
frame.cy = cab.depthY/2 + cfg_frameGap + frame.depthY/2;
conv.cx = cab.widthX/2 + cfg_convGap + conv.widthX/2;
conv.cy = cfg_convOffY;
convSurfZ   = conv.heightZ + conv.rollerR + conv.beltH;
palletSurfZ = pallet.heightZ;
Tbase = eye(4); Tbase(1,4)=baseX; Tbase(2,4)=baseY; Tbase(3,4)=baseZ;

% === 动态计算环境碰撞体坐标 (机器人基坐标系, m, 与C++ v5.0一致) ===
frmHW = frame.widthX/2;
frmCY = frame.cy;
frmNY = frmCY - frame.depthY/2;
frmFY = frmCY + frame.depthY/2;
frmZB = -baseZ;      % 底部 (世界Z=0 → 基坐标Z=-baseZ)
frmZT = frame.height - baseZ;  % 顶部
frmR = frame.tubeR + 0.020;    % 碰撞半径 = 管径+20mm = 50mm (匹配C++ FRM_TUBE_R+20)
ENV_COLL.frameColumns = [  % [x,y,z1,z2,r] (m, 基坐标系)
    -frmHW, frmNY, frmZB, frmZT, frmR;
     frmHW, frmNY, frmZB, frmZT, frmR;
    -frmHW, frmFY, frmZB, frmZT, frmR;
     frmHW, frmFY, frmZB, frmZT, frmR;
];
% 框架顶梁 (envId 20-21, 平行于Y轴, 连接近端→远端立柱顶部)
ENV_COLL.frameTopBars = [  % [x1,y1,z1, x2,y2,z2, r]
    -frmHW, frmNY, frmZT, -frmHW, frmFY, frmZT, frmR;   % 左侧顶梁 (Y平行)
     frmHW, frmNY, frmZT,  frmHW, frmFY, frmZT, frmR;   % 右侧顶梁 (Y平行)
];
% 电箱碰撞体 (4条边, 基坐标系)
cabHW = cab.widthX/2; cabHD = cab.depthY/2;
cabZB = frmZB; cabZT = -0.08;  % 电箱顶略低于机器人基座
ENV_COLL.cabinet = [  % [x1,y1,z1, x2,y2,z2, r]
    -cabHW,-cabHD,cabZB, -cabHW, cabHD,cabZT, ENV_COLL.cabR_m;
     cabHW,-cabHD,cabZB,  cabHW, cabHD,cabZT, ENV_COLL.cabR_m;
    -cabHW,-cabHD,cabZB,  cabHW,-cabHD,cabZT, ENV_COLL.cabR_m;
    -cabHW, cabHD,cabZB,  cabHW, cabHD,cabZT, ENV_COLL.cabR_m;
];
% 传送带碰撞体 (3条: 左右侧板+皮带面)
cvX = conv.cx; cvY = conv.cy; cvHL = conv.lengthY/2; cvHW = conv.widthX/2;
cvSZ = convSurfZ - baseZ;
ENV_COLL.conveyor = [  % [x1,y1,z1, x2,y2,z2, r]
    cvX-cvHW, cvY-cvHL, -0.2,  cvX-cvHW, cvY+cvHL, -0.2, ENV_COLL.convR_m;
    cvX+cvHW, cvY-cvHL, -0.2,  cvX+cvHW, cvY+cvHL, -0.2, ENV_COLL.convR_m;
    cvX,      cvY-cvHL, cvSZ-0.02, cvX,  cvY+cvHL, cvSZ-0.02, cvHW;
];
fprintf('  ENV_COLL: frame(4col+2topY)=[%.3f,%.3f]->[%.3f,%.3f] cab(4) conv(3)\n', ...
    frmNY, frmFY, frmZB, frmZT);

fprintf('\n');
fprintf([char(9556) repmat(char(9552),1,72) char(9559) '\n']);
fprintf([char(9553) '  HR_S50-2000 v15.0 -- Precise Layout + Full Env Collision 3D Sim    ' char(9553) '\n']);
fprintf([char(9562) repmat(char(9552),1,72) char(9565) '\n\n']);
fprintf('Font: %s | Headless: %d | nBoxes: %d\n', CJK_FONT, isHeadless, nBoxes);
fprintf('Collision .so: %s\n', SO_PATH);

timing = struct();
timing.soInit_ms = 0;
timing.meshLoad_ms = 0;
timing.dataLoad_ms = 0;
timing.collisionCheck_ms = 0;
timing.fkCompute_ms = 0;
timing.tcpAnalysis_ms = 0;
timing.rendering_ms = 0;
timing.animation_ms = 0;
timing.totalFigures_ms = 0;
timing.collisionCalls = 0;
timing.fkCalls = 0;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║              加载碰撞检测.so库 (HansAlgorithmExport)                ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n--- Initializing Collision Detection .so ---\n');
tSoInit = tic;

soLoaded = false;
try
    if libisloaded('libHRCInterface')
        unloadlibrary('libHRCInterface');
    end
    
    if ~exist(SO_PATH, 'file')
        error('碰撞.so不存在: %s', SO_PATH);
    end
    if ~exist(SO_HEADER, 'file')
        error('头文件不存在: %s', SO_HEADER);
    end
    
    % 预加载stubs库 (解决libHRCInterface.so中的initTCPPosition undefined symbol)
    % MATLAB的loadlibrary使用RTLD_LOCAL, 需要用RTLD_GLOBAL才能让符号对后续库可见
    stubsPath = fullfile(fileparts(mfilename('fullpath')), 's50_tcp_stubs.so');
    if exist(stubsPath, 'file')
        try
            dlopen_global(stubsPath);  % MEX: dlopen(path, RTLD_GLOBAL|RTLD_LAZY)
        catch stubErr
            fprintf('  stubs预加载失败: %s\n', stubErr.message);
            fprintf('  备选方案: LD_PRELOAD=%s matlab ...\n', stubsPath);
        end
    else
        fprintf('  ⚠ s50_tcp_stubs.so 未找到\n');
    end
    
    [~, warnings] = loadlibrary(SO_PATH, SO_HEADER, 'alias', 'libHRCInterface');
    if ~isempty(warnings)
        fprintf('  loadlibrary warnings: %s\n', warnings);
    end
    
    dh = DH_MM;
    baseGeo     = COLLISION_GEO.base;
    lowerArmGeo = COLLISION_GEO.lowerArm;
    elbowGeo    = COLLISION_GEO.elbow;
    upperArmGeo = COLLISION_GEO.upperArm;
    wristGeo    = COLLISION_GEO.wrist;
    initJoint   = [0, -90, 0, 0, 90, 0];
    
    % v1.0.0 必须先调用 initilizeRobotType + setKinParams
    calllib('libHRCInterface', 'initilizeRobotType', int32(1));
    kinParams = [DH_MM, 0, 0];  % 10-element: DH[8] + {0, 0}
    calllib('libHRCInterface', 'setKinParams', kinParams);
    calllib('libHRCInterface', 'initACAreaConstrainPackageInterface', ...
        int32(1), dh, baseGeo, lowerArmGeo, elbowGeo, upperArmGeo, wristGeo, initJoint);
    
    flags = int8([1, 1, 1]);
    calllib('libHRCInterface', 'setCPSelfColliderLinkModelOpenStateInterface', flags);
    
    vel = zeros(1,6); acc = zeros(1,6);
    calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', ...
        initJoint, vel, acc);
    
    pairArr = int64([0, 0]);
    distVal = 0.0;
    [collResult, pairArr, distVal] = calllib('libHRCInterface', ...
        'checkCPSelfCollisionInterface', pairArr, distVal);
    
    soLoaded = true;
    fprintf('  ✅ libHRCInterface.so 加载成功\n');
    fprintf('  HOME验证: collision=%d, min_dist=%.1f mm, pair=[%d,%d]\n', ...
        collResult, distVal, pairArr(1), pairArr(2));
    
    % === 环境碰撞配置 (匹配C++ testS50PalletizingSO.cpp 阶段2) ===
    fprintf('\n  --- 环境碰撞配置 ---\n');
    soStat = {'❌','✅'};  % result==0 → ✅
    envRegOK = 0;  % 成功注册计数
    
    % 开启全部连杆-环境碰撞检测 (7个连杆)
    linkEnvFlags = int8([1,1,1,1,1,1,1]);
    calllib('libHRCInterface', 'setLinkEnvCollisionEnabledInterface', linkEnvFlags);
    fprintf('    连杆-环境碰撞: 7/7 已开启\n');
    
    % 框架立柱 (envId 1-4, 胶囊体, 匹配C++ frmIds)
    for ci = 1:size(ENV_COLL.frameColumns, 1)
        fc = ENV_COLL.frameColumns(ci,:);
        startPt = [fc(1), fc(2), fc(3)] * 1000;  % m→mm
        endPt   = [fc(1), fc(2), fc(4)] * 1000;
        r_mm    = fc(5) * 1000;
        envId   = int64(ci);  % envId 1,2,3,4
        result  = calllib('libHRCInterface', 'addEnvObstacleCapsuleInterface', envId, startPt, endPt, r_mm);
        envRegOK = envRegOK + (result == 0);
        fprintf('    框架柱 envId=%d: %s (r=%.0fmm)\n', envId, soStat{(result==0)+1}, r_mm);
    end
    
    % 框架顶梁 (envId 20-21)
    for ci = 1:size(ENV_COLL.frameTopBars, 1)
        tb = ENV_COLL.frameTopBars(ci,:);
        startPt = [tb(1), tb(2), tb(3)] * 1000;
        endPt   = [tb(4), tb(5), tb(6)] * 1000;
        r_mm    = tb(7) * 1000;
        envId   = int64(19 + ci);  % envId 20,21
        result  = calllib('libHRCInterface', 'addEnvObstacleCapsuleInterface', envId, startPt, endPt, r_mm);
        envRegOK = envRegOK + (result == 0);
        fprintf('    框架顶梁 envId=%d: %s\n', envId, soStat{(result==0)+1});
    end
    
    % 电箱 (envId 10-13, 胶囊体)
    for ci = 1:size(ENV_COLL.cabinet, 1)
        cc = ENV_COLL.cabinet(ci,:);
        startPt = [cc(1), cc(2), cc(3)] * 1000;
        endPt   = [cc(4), cc(5), cc(6)] * 1000;
        r_mm    = cc(7) * 1000;
        envId   = int64(9 + ci);  % envId 10,11,12,13
        result  = calllib('libHRCInterface', 'addEnvObstacleCapsuleInterface', envId, startPt, endPt, r_mm);
        envRegOK = envRegOK + (result == 0);
        fprintf('    电箱 envId=%d: %s\n', envId, soStat{(result==0)+1});
    end
    
    % 传送带 (envId 15-17, 胶囊体)
    for ci = 1:size(ENV_COLL.conveyor, 1)
        cv = ENV_COLL.conveyor(ci,:);
        startPt = [cv(1), cv(2), cv(3)] * 1000;
        endPt   = [cv(4), cv(5), cv(6)] * 1000;
        r_mm    = cv(7) * 1000;
        envId   = int64(14 + ci);  % envId 15,16,17
        result  = calllib('libHRCInterface', 'addEnvObstacleCapsuleInterface', envId, startPt, endPt, r_mm);
        envRegOK = envRegOK + (result == 0);
        fprintf('    传送带 envId=%d: %s\n', envId, soStat{(result==0)+1});
    end
    
    % 验证障碍物数量
    envCount = calllib('libHRCInterface', 'getEnvObstacleCountInterface');
    fprintf('    注册成功: %d/%d, SO报告障碍物: %d\n', envRegOK, ...
        size(ENV_COLL.frameColumns,1)+size(ENV_COLL.frameTopBars,1)+ ...
        size(ENV_COLL.cabinet,1)+ ...
        size(ENV_COLL.conveyor,1), envCount);
    
catch ME
    fprintf('  ⚠️ .so加载失败: %s\n', ME.message);
    fprintf('  将使用C++预计算碰撞数据作为备选\n');
    soLoaded = false;
end
timing.soInit_ms = toc(tSoInit)*1000;
fprintf('  初始化耗时: %.2f ms\n', timing.soInit_ms);

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                    加载 STL 网格 (含降面优化)                       ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n--- Loading STL Meshes (reduction=%.0f%%) ---\n', MESH_REDUCE_RATIO*100);
tMesh = tic;

meshNames = {'elfin_base','elfin_link1','elfin_link2','elfin_link3',...
             'elfin_link4','elfin_link5','elfin_link6'};
meshData = cell(7,1);
meshDataLow = cell(7,1);
totalVerts = 0; totalFaces = 0;
totalVertsLow = 0; totalFacesLow = 0;

for i = 1:7
    fn = fullfile(meshDir, [meshNames{i}, '.STL']);
    if ~exist(fn, 'file'), error('STL not found: %s', fn); end
    tr = stlread(fn);
    
    maxCoord = max(abs(tr.Points(:)));
    if maxCoord > 10, sc = 0.001; else, sc = 1.0; end
    
    V = tr.Points * sc;
    F = tr.ConnectivityList;
    meshData{i}.V = V; meshData{i}.F = F;
    totalVerts = totalVerts + size(V,1);
    totalFaces = totalFaces + size(F,1);
    
    nf = size(F,1);
    targetFaces = max(200, round(nf * MESH_REDUCE_RATIO));
    [Flo, Vlo] = reducepatch(F, V, targetFaces);
    meshDataLow{i}.V = Vlo; meshDataLow{i}.F = Flo;
    totalVertsLow = totalVertsLow + size(Vlo,1);
    totalFacesLow = totalFacesLow + size(Flo,1);
    
    fprintf('  %-16s: %5d→%4d faces (%.0f%% reduction)\n', ...
        meshNames{i}, nf, size(Flo,1), (1-size(Flo,1)/nf)*100);
end
timing.meshLoad_ms = toc(tMesh)*1000;
fprintf('  原始: %d verts, %d faces\n', totalVerts, totalFaces);
fprintf('  低模: %d verts, %d faces (动画用)\n', totalVertsLow, totalFacesLow);
fprintf('  加载耗时: %.1f ms\n\n', timing.meshLoad_ms);

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                    加载C++轨迹数据                                  ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('--- Loading C++ Trajectory Data ---\n');
tData = tic;

pall_raw = loadNumericData(fullfile(dataDir, 'so_palletizing_trajectory.txt'));
fprintf('  SO palletizing: %d rows x %d cols\n', size(pall_raw,1), size(pall_raw,2));

coll_raw = [];
collTrajFile = fullfile(dataDir, 'so_collision_trajectory.txt');
if exist(collTrajFile, 'file')
    coll_raw = loadNumericData(collTrajFile);
    fprintf('  SO collision:   %d rows x %d cols\n', size(coll_raw,1), size(coll_raw,2));
else
    fprintf('  SO collision:   (not found, skip)\n');
end

pallSummary = readSummaryFile(fullfile(dataDir, 'so_palletizing_summary.txt'));

collSummary = struct();
collSummFile = fullfile(dataDir, 'so_collision_summary.txt');
if exist(collSummFile, 'file')
    collSummary = readSummaryFile(collSummFile);
end

% 加载CSV profile (v5.0: 16列含envCollision)
profileData = [];
profileFile = fullfile(dataDir, 'so_palletizing_profile.csv');
if exist(profileFile, 'file')
    try
        fid = fopen(profileFile, 'r');
        headerLine = fgetl(fid); % skip header
        profileData = [];
        while ~feof(fid)
            line = fgetl(fid);
            if ischar(line) && ~isempty(strtrim(line))
                vals = sscanf(line, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%f,%f,%f');
                if length(vals) == 16
                    profileData = [profileData; vals']; %#ok<AGROW>
                end
            end
        end
        fclose(fid);
        fprintf('  CSV profile:    %d rows x 16 cols\n', size(profileData,1));
    catch ME2
        fprintf('  CSV profile:    load failed: %s\n', ME2.message);
    end
end

timing.dataLoad_ms = toc(tData)*1000;
fprintf('  加载耗时: %.1f ms\n', timing.dataLoad_ms);

% v5.0 summary字段解析
selfCollisions = str2double(getField(pallSummary, 'self_collisions', ...
                    getField(pallSummary, 'collisions', '0')));
envCollisions  = str2double(getField(pallSummary, 'env_collisions', '0'));
taskOrder      = getField(pallSummary, 'order', 'unknown');
envObstacles   = getField(pallSummary, 'env_obstacles', 'none');
toolCollision  = getField(pallSummary, 'tool_collision', 'none');

fprintf('\n  v5.0 Stats: self=%d env=%d order=%s\n', selfCollisions, envCollisions, taskOrder);
fprintf('  Env obstacles: %s | Tool: %s\n', envObstacles, toolCollision);

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║              实时碰撞验证 (.so vs C++ 预计算)                       ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n--- Real-time Collision Verification (.so) ---\n');

hasColl = ~isempty(coll_raw);
if hasColl
    coll_scenarios = unique(coll_raw(:,1));
    nScenarios = length(coll_scenarios);
    coll_keyQ = zeros(nScenarios,6);
    coll_minD_cpp = zeros(nScenarios,1);
    coll_minD_so = zeros(nScenarios,1);
    coll_tcp_cpp  = zeros(nScenarios,3);
    coll_tcp_so   = zeros(nScenarios,6);
    
    for si = 1:nScenarios
        mask = coll_raw(:,1)==coll_scenarios(si);
        rows = coll_raw(mask,:);
        [minD, idx] = min(rows(:,21));
        coll_keyQ(si,:) = rows(idx, 3:8);
        coll_minD_cpp(si) = minD;
        if size(rows,2) >= 24
            coll_tcp_cpp(si,:) = rows(idx, 22:24);
        end
    end
    
    if soLoaded
        tCollVerify = tic;
        for si = 1:nScenarios
            q_deg = coll_keyQ(si,:);
            vel = zeros(1,6); acc = zeros(1,6);
            calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', q_deg, vel, acc);
            pairArr = int64([0, 0]); distVal = 0.0;
            [~, pairArr, distVal] = calllib('libHRCInterface', 'checkCPSelfCollisionInterface', pairArr, distVal);
            coll_minD_so(si) = distVal;
            timing.collisionCalls = timing.collisionCalls + 1;
            
            tcpS = libstruct('MC_COORD_REF');
            tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
            [~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);
            coll_tcp_so(si,:) = [tcpS.X, tcpS.Y, tcpS.Z, tcpS.A, tcpS.B, tcpS.C];
             timing.fkCalls = timing.fkCalls + 1;
        end
        timing.collisionCheck_ms = timing.collisionCheck_ms + toc(tCollVerify)*1000;
        
        fprintf('\n  C++ vs .so 碰撞距离对比 (%d scenarios)\n', nScenarios);
        for si = 1:min(nScenarios,7)
            diff = abs(coll_minD_cpp(si) - coll_minD_so(si));
            if diff < 1, marker = '✅'; elseif diff < 10, marker = '⚠️'; else, marker = '❌'; end
            fprintf('    S%-3d: C++=%.1f .so=%.1f %s%.1f\n', ...
                coll_scenarios(si), coll_minD_cpp(si), coll_minD_so(si), marker, diff);
        end
    else
        coll_minD_so = coll_minD_cpp;
        fprintf('  跳过.so验证 (库未加载)\n');
    end
else
    nScenarios = 0;
    coll_scenarios = [];
    coll_keyQ = [];
    coll_minD_cpp = [];
    coll_minD_so = [];
    coll_tcp_so = [];
    fprintf('  无碰撞轨迹数据\n');
end

% 码垛任务关键姿态
pall_tasks = unique(pall_raw(:,1));
nTasks = length(pall_tasks);
pall_keyQ = zeros(nTasks,6);
pall_minD = zeros(nTasks,1);
pall_tcp  = zeros(nTasks,3);
pall_minD_so = zeros(nTasks,1);
pall_tcp_so  = zeros(nTasks,6);

for ti = 1:nTasks
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows = pall_raw(mask,:);
    [minD,~] = min(rows(:,16));
    pall_keyQ(ti,:) = rows(end, 4:9);
    pall_minD(ti) = minD;
    if size(rows,2) >= 19
        pall_tcp(ti,:) = rows(end, 17:19);
    end
end

if soLoaded
    tPallVerify = tic;
    for ti = 1:nTasks
        q_deg = pall_keyQ(ti,:);
        vel = zeros(1,6); acc = zeros(1,6);
        calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', q_deg, vel, acc);
        pairArr = int64([0,0]); distVal = 0.0;
        [~, pairArr, distVal] = calllib('libHRCInterface', 'checkCPSelfCollisionInterface', pairArr, distVal);
        pall_minD_so(ti) = distVal;
        timing.collisionCalls = timing.collisionCalls + 1;
        
        tcpS = libstruct('MC_COORD_REF');
        tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
        [~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);
        pall_tcp_so(ti,:) = [tcpS.X, tcpS.Y, tcpS.Z, tcpS.A, tcpS.B, tcpS.C];
        timing.fkCalls = timing.fkCalls + 1;
    end
    timing.collisionCheck_ms = timing.collisionCheck_ms + toc(tPallVerify)*1000;
    fprintf('\n  码垛.so验证: %d tasks, min_dist range [%.1f, %.1f] mm\n', ...
        nTasks, min(pall_minD_so), max(pall_minD_so));
else
    pall_minD_so = pall_minD;
    pall_tcp_so = [pall_tcp, zeros(nTasks,3)];  % C++输出mm, FK2 v1.0.0也返回mm
end

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║              全轨迹实时碰撞扫描 + TCP姿态分析                       ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n--- Full Trajectory Collision Scan + TCP Pose Analysis ---\n');

SCAN_STRIDE = 5;
nPall = size(pall_raw,1);
so_pall_dist = nan(nPall,1);
so_pall_tcp  = nan(nPall,6);
so_pall_tcpAxis = nan(nPall,3);

% 提前计算task→box映射 (扫描和动画都需要)
scanBoxForTask = min((1:nTasks)', nBoxes);

if soLoaded
    tScan = tic;
    scanCount = 0;
    scanPrevTask = -1; scanPrevSeg = -1; scanToolActive = false;
    scanLastTcp_mm = [0 0 0];  % 记录seg5末TCP位置作为placed box中心
    for ri = 1:SCAN_STRIDE:nPall
        q_deg = pall_raw(ri, 4:9);
        vel   = pall_raw(ri, 10:15);
        acc = zeros(1,6);
        
        % 段切换时管理工具碰撞球 + 已放置箱子 (匹配C++三阶段管理)
        curTask = pall_raw(ri, 1); curSeg = pall_raw(ri, 2);
        if curTask ~= scanPrevTask || curSeg ~= scanPrevSeg
            tidx = find(pall_tasks == curTask, 1);
            if ~isempty(tidx), sbi = scanBoxForTask(tidx); else, sbi = 0; end
            if curSeg == 2 && ~scanToolActive && sbi > 0 && sbi <= nBoxes
                calllib('libHRCInterface', 'setCPToolCollisionBallShapeInterface', ...
                    int64(6), [0, 0, -box.hz/2*1000], 225.0);
                scanToolActive = true;
            end
            if curSeg == 6 && scanToolActive
                calllib('libHRCInterface', 'removeCPToolCollisonInterface', int64(6));
                scanToolActive = false;
                % 使用上一步FK2 TCP位置(m→mm)作为placed box center
                if sbi > 0 && sbi <= nBoxes
                    calllib('libHRCInterface', 'addEnvObstacleBallInterface', ...
                        int64(45 + sbi), scanLastTcp_mm, 250.0);
                end
            end
            scanPrevTask = curTask; scanPrevSeg = curSeg;
        end
        
        calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', q_deg, vel, acc);
        pairArr = int64([0,0]); distVal = 0.0;
        [~, pairArr, distVal] = calllib('libHRCInterface', 'checkCPSelfCollisionInterface', pairArr, distVal);
        so_pall_dist(ri) = distVal;
        
        tcpS = libstruct('MC_COORD_REF');
        tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
        [~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);
        so_pall_tcp(ri,:) = [tcpS.X, tcpS.Y, tcpS.Z, tcpS.A, tcpS.B, tcpS.C];
        scanLastTcp_mm = [tcpS.X, tcpS.Y, tcpS.Z];  % FK2 v1.0.0 已返回mm
        
        % TCP Z轴: 从FK2 A,B,C (ZYX Euler deg) 计算 (避免混用urdfFK)
        % R = Rz(A)*Ry(B)*Rx(C), Z-axis = R*[0;0;1]
        Ar = deg2rad(tcpS.A); Br = deg2rad(tcpS.B); Cr = deg2rad(tcpS.C);
        so_pall_tcpAxis(ri,:) = [...
            cos(Ar)*sin(Br)*cos(Cr) + sin(Ar)*sin(Cr), ...
            sin(Ar)*sin(Br)*cos(Cr) - cos(Ar)*sin(Cr), ...
            cos(Br)*cos(Cr)];
        
        scanCount = scanCount + 1;
        timing.collisionCalls = timing.collisionCalls + 1;
        timing.fkCalls = timing.fkCalls + 1;
    end
    % 扫描结束后清理工具碰撞球 (确保后续操作干净)
    if scanToolActive
        calllib('libHRCInterface', 'removeCPToolCollisonInterface', int64(6));
    end
    % 清除扫描时添加的已放置箱子障碍 (后续动画会重新管理)
    for bi2 = 1:nBoxes
        calllib('libHRCInterface', 'removeEnvObstacleInterface', int64(45 + bi2));
    end
    scanTime_ms = toc(tScan)*1000;
    timing.collisionCheck_ms = timing.collisionCheck_ms + scanTime_ms;
    
    validIdx = find(~isnan(so_pall_dist));
    so_pall_dist = interp1(validIdx, so_pall_dist(validIdx), (1:nPall)', 'linear', 'extrap');
    for ci = 1:6
        vals = so_pall_tcp(validIdx, ci);
        so_pall_tcp(:,ci) = interp1(validIdx, vals, (1:nPall)', 'linear', 'extrap');
    end
    for ci = 1:3
        vals = so_pall_tcpAxis(validIdx, ci);
        so_pall_tcpAxis(:,ci) = interp1(validIdx, vals, (1:nPall)', 'linear', 'extrap');
    end
    
    fprintf('  扫描 %d/%d 点, 耗时 %.1f ms (%.1f us/call)\n', ...
        scanCount, nPall, scanTime_ms, scanTime_ms/scanCount*1000);
    
    tcpAxisDot = so_pall_tcpAxis * TCP_DESIRED_AXIS;
    tcpOrientError_deg = acosd(max(-1, min(1, tcpAxisDot)));
    fprintf('  TCP姿态偏差: mean=%.1f°, max=%.1f°\n', ...
        mean(tcpOrientError_deg), max(tcpOrientError_deg));
else
    so_pall_dist = pall_raw(:,16);
    fprintf('  跳过.so扫描 (使用C++预计算数据)\n');
    tcpOrientError_deg = [];
end

% 碰撞轨迹扫描
nColl = size(coll_raw,1);
so_coll_dist = nan(nColl,1);
if soLoaded && hasColl
    tCScan = tic;
    for ri = 1:SCAN_STRIDE:nColl
        q_deg = coll_raw(ri, 3:8);
        vel   = coll_raw(ri, 9:14);
        acc   = coll_raw(ri, 15:20);
        calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', q_deg, vel, acc);
        pairOut = zeros(1,2,'int64'); distOut = 0.0;
        [~, ~, distOut] = calllib('libHRCInterface', 'checkCPSelfCollisionInterface', pairOut, distOut);
        so_coll_dist(ri) = distOut;
        timing.collisionCalls = timing.collisionCalls + 1;
    end
    validIdx2 = find(~isnan(so_coll_dist));
    so_coll_dist = interp1(validIdx2, so_coll_dist(validIdx2), (1:nColl)', 'linear', 'extrap');
    fprintf('  碰撞轨迹扫描: %d 点, %.1f ms\n', length(validIdx2), toc(tCScan)*1000);
elseif hasColl
    so_coll_dist = coll_raw(:,21);
end

startTime = tic;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 1: 碰撞场景 STL 姿态 (如有碰撞数据)                        ║
%% ╚══════════════════════════════════════════════════════════════════════╝
if hasColl
    fprintf('\n>>> Fig 1: Collision scenario STL poses...\n');
    tFig = tic;
    fig1 = figure('Position',[20 20 1920 1080],'Color','w','Name','Collision STL Poses (v15)');
    nPlots = min(nScenarios,7);
    for si = 1:nPlots
        ax = subplot(2,4,si,'Parent',fig1);
        q_rad = deg2rad(coll_keyQ(si,:));
        renderSTLRobot(ax, meshData, JOINTS, q_rad, LINK_COLORS, LINK_ALPHA);
        
        dist_show = coll_minD_so(si);
        if dist_show>200, tc=[0 0.6 0.2];
        elseif dist_show>100, tc=[0.8 0.6 0];
        else, tc=[0.9 0.1 0.1]; end
        
        title(ax, sprintf('S%d | d=%.1fmm', coll_scenarios(si), dist_show), ...
            'FontSize',10,'FontWeight','bold','Color',tc,'FontName',CJK_FONT);
        view(ax,135,25);
    end
    ax_sum = subplot(2,4,8,'Parent',fig1);
    hold(ax_sum,'on');
    if soLoaded
        b1 = barh(ax_sum, [coll_minD_cpp(1:nPlots), coll_minD_so(1:nPlots)]);
        b1(1).FaceColor = [0.7 0.7 0.7]; b1(2).FaceColor = [0.3 0.8 0.4];
        legend(ax_sum, {'C++','.so'}, 'FontSize',8);
    else
        barh(ax_sum, coll_minD_so(1:nPlots), 'FaceColor', [0.3 0.8 0.4]);
    end
    set(ax_sum,'YTick',1:nPlots,'YTickLabel',arrayfun(@(x)sprintf('S%d',x),coll_scenarios(1:nPlots),'Un',0));
    xlabel(ax_sum,'Min Self-Distance (mm)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    title(ax_sum,'Safety Summary','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
    grid(ax_sum,'on');
    sgtitle(fig1,'HR\_S50-2000 碰撞场景 (v15)',...
        'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
    saveFig(fig1, outputDir, '01_collision_poses_so');
    timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;
end

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 2: 环境碰撞Profile + 自碰撞距离分析                        ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n>>> Fig 2: Env collision profile + self-distance analysis...\n');
tFig = tic;
fig2 = figure('Position',[20 20 1920 1080],'Color','w','Name','Collision Profile v15');

% 2a: 码垛轨迹自碰撞距离
ax2a = subplot(2,3,1,'Parent',fig2);
hold(ax2a,'on');
plot(ax2a, 1:nPall, pall_raw(:,16), '-','Color',[0.7 0.7 0.7],'LineWidth',1.5);
if soLoaded
    plot(ax2a, 1:nPall, so_pall_dist, '-','Color',[0.2 0.6 0.9],'LineWidth',1.5);
    legend(ax2a,{'C++ precomputed','.so realtime'},'FontSize',8,'Location','best');
end
xlabel(ax2a,'Sample','FontSize',10,'FontName',CJK_FONT);
ylabel(ax2a,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax2a,'码垛: 自碰撞距离曲线','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax2a,'on');

% 2b: CSV环境碰撞标记可视化
ax2b = subplot(2,3,2,'Parent',fig2);
if ~isempty(profileData)
    hold(ax2b,'on');
    selfDist = profileData(:,11);
    envColl  = profileData(:,13);
    selfColl = profileData(:,12);
    
    plot(ax2b, 1:length(selfDist), selfDist, '-','Color',[0.5 0.5 0.5],'LineWidth',0.8);
    
    % 标记自碰撞点
    selfIdx = find(selfColl > 0);
    if ~isempty(selfIdx)
        plot(ax2b, selfIdx, selfDist(selfIdx), 'r.', 'MarkerSize', 4);
    end
    
    % 标记环境碰撞点
    envIdx = find(envColl > 0);
    if ~isempty(envIdx)
        plot(ax2b, envIdx, selfDist(envIdx), 'bs', 'MarkerSize', 6, 'LineWidth', 2);
    end
    
    xlabel(ax2b,'CSV Step','FontSize',10,'FontName',CJK_FONT);
    ylabel(ax2b,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
    title(ax2b,sprintf('CSV Profile: self=%d env=%d', sum(selfColl>0), sum(envColl>0)),...
        'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    legend(ax2b,{'Self-Dist','Self-Coll','Env-Coll'},'FontSize',8,'Location','best');
    grid(ax2b,'on');
else
    text(ax2b,0.5,0.5,'CSV Profile不可用','FontSize',12,'HorizontalAlignment','center','FontName',CJK_FONT);
    axis(ax2b,'off');
end

% 2c: 碰撞距离分布
ax2c = subplot(2,3,3,'Parent',fig2);
if soLoaded
    diffPall = so_pall_dist - pall_raw(:,16);
    histogram(ax2c, diffPall, 50, 'FaceColor',[0.3 0.7 0.5],'EdgeColor','none');
    xlabel(ax2c,'\Delta dist: .so - C++','FontSize',10,'FontName',CJK_FONT);
    title(ax2c,sprintf('差异分布 (mean=%.2f, std=%.2f)', mean(diffPall), std(diffPall)),...
        'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
else
    histogram(ax2c, so_pall_dist, 50, 'FaceColor',[0.3 0.6 0.9],'EdgeColor','none');
    xlabel(ax2c,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
    title(ax2c,'碰撞距离分布','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
end
ylabel(ax2c,'Count','FontSize',10,'FontName',CJK_FONT);
grid(ax2c,'on');

% 2d: TCP姿态偏差
ax2d = subplot(2,3,4,'Parent',fig2);
if ~isempty(tcpOrientError_deg)
    hold(ax2d,'on');
    plot(ax2d, 1:nPall, tcpOrientError_deg, '-','Color',[0.8 0.3 0.1],'LineWidth',1);
    yline(ax2d, TCP_ORIENT_TOL_DEG, 'r--','LineWidth',1.5,'Label',sprintf('容差%.0f°',TCP_ORIENT_TOL_DEG));
    xlabel(ax2d,'Sample','FontSize',10,'FontName',CJK_FONT);
    ylabel(ax2d,'TCP Error (deg)','FontSize',10,'FontName',CJK_FONT);
    title(ax2d,'TCP姿态偏差','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    grid(ax2d,'on');
else
    text(ax2d,0.5,0.5,'TCP数据不可用','FontSize',12,'HorizontalAlignment','center','FontName',CJK_FONT);
    axis(ax2d,'off');
end

% 2e: 每任务安全裕度
ax2e = subplot(2,3,5,'Parent',fig2);
b = bar(ax2e, pall_minD_so, 'FaceColor','flat');
for ti=1:nTasks
    if pall_minD_so(ti)>400, b.CData(ti,:)=[0.3 0.8 0.4];
    elseif pall_minD_so(ti)>200, b.CData(ti,:)=[1.0 0.8 0.3];
    else, b.CData(ti,:)=[0.9 0.3 0.3]; end
end
set(ax2e,'XTick',1:nTasks,'XTickLabel',arrayfun(@(x)sprintf('T%d',x),1:nTasks,'Un',0));
ylabel(ax2e,'Min Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax2e,sprintf('码垛安全裕度 (%d tasks)', nTasks),'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax2e,'on');

% 2f: v5.0 碰撞统计摘要卡
ax2f = subplot(2,3,6,'Parent',fig2); axis(ax2f,'off');
xlim(ax2f,[0 1]); ylim(ax2f,[0 1]); hold(ax2f,'on');
rectangle(ax2f,'Position',[0.02 0.02 0.96 0.96],'FaceColor',[0.97 0.97 1.0],...
    'EdgeColor',[0.3 0.3 0.6],'LineWidth',2,'Curvature',0.05);
yP = 0.92;
text(ax2f,0.50,yP,'v5.0 碰撞统计','FontSize',14,'FontWeight','bold',...
    'HorizontalAlignment','center','Color',[0.1 0.1 0.3],'FontName',CJK_FONT);
yP = yP - 0.08;
if envCollisions == 0, envc=[0 0.6 0.2]; else, envc=[0.9 0.1 0.1]; end
summLines = {
    sprintf('自碰撞: %d 次', selfCollisions), [0.9 0.4 0.1];
    sprintf('环境碰撞: %d 次', envCollisions), envc;
    '', [0 0 0];
    sprintf('顺序: %s', taskOrder), [0.2 0.2 0.5];
    sprintf('环境障碍: %s', envObstacles), [0.2 0.2 0.5];
    sprintf('工具碰撞体: %s', toolCollision), [0.2 0.2 0.5];
    '', [0 0 0];
    sprintf('最小距离: %s mm', getField(pallSummary,'min_dist_mm','?')), [0.5 0.2 0.1];
    sprintf('规划耗时: %s ms', getField(pallSummary,'planning_total_ms','?')), [0.2 0.4 0.6];
    sprintf('总计算: %s s', getField(pallSummary,'elapsed_s','?')), [0.2 0.4 0.6];
};
for si = 1:size(summLines,1)
    if isempty(summLines{si,1}), yP = yP-0.03; continue; end
    text(ax2f, 0.10, yP, summLines{si,1}, 'FontSize',11, 'FontWeight','bold', ...
        'Color',summLines{si,2}, 'FontName',CJK_FONT);
    yP = yP - 0.06;
end

sgtitle(fig2,'v15 碰撞分析: 环境碰撞 + 自碰撞 + TCP感知',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig2, outputDir, '02_collision_profile_v4');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 3: 碰撞模型可视化 + 环境碰撞体                             ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 3: Collision geometry + env obstacles...\n');
tFig = tic;
fig3 = figure('Position',[20 20 1920 1080],'Color','w','Name','Collision Geometry v15');

% 3a: 静态全场景 + 环境碰撞体
ax3a = subplot(1,2,1,'Parent',fig3);
hold(ax3a,'on');
% SO碰撞体渲染 (真实DH坐标系, v1.0.0 getUIInfoMation)
renderCapsuleRobotHandles(ax3a, [0,-90,0,0,90,0], [baseX,baseY,baseZ], JOINTS);

% 绘制环境碰撞体: 框架柱胶囊 (红色半透明)
for ci = 1:size(ENV_COLL.frameColumns, 1)
    fc = ENV_COLL.frameColumns(ci,:);
    p1 = [fc(1), fc(2), fc(3)] + [baseX, baseY, baseZ];  % 转换到世界坐标系
    p2 = [fc(1), fc(2), fc(4)] + [baseX, baseY, baseZ];
    drawCapsule3D(ax3a, p1, p2, fc(5), [0.9 0.2 0.2], 0.20);
end
% 框架顶梁 (黄色半透明)
for ci = 1:size(ENV_COLL.frameTopBars, 1)
    tb = ENV_COLL.frameTopBars(ci,:);
    p1 = [tb(1), tb(2), tb(3)] + [baseX, baseY, baseZ];
    p2 = [tb(4), tb(5), tb(6)] + [baseX, baseY, baseZ];
    drawCapsule3D(ax3a, p1, p2, tb(7), [0.9 0.7 0.1], 0.18);
end

% 绘制环境碰撞体: 电气柜胶囊 (橙色半透明)
for ci = 1:size(ENV_COLL.cabinet, 1)
    cc = ENV_COLL.cabinet(ci,:);
    p1 = [cc(1), cc(2), cc(3)] + [baseX, baseY, baseZ];
    p2 = [cc(4), cc(5), cc(6)] + [baseX, baseY, baseZ];
    drawCapsule3D(ax3a, p1, p2, cc(7), [0.8 0.6 0.2], 0.15);
end

% 绘制环境碰撞体: 传送带胶囊 (灰色半透明)
for ci = 1:size(ENV_COLL.conveyor, 1)
    cv = ENV_COLL.conveyor(ci,:);
    p1 = [cv(1), cv(2), cv(3)] + [baseX, baseY, baseZ];
    p2 = [cv(4), cv(5), cv(6)] + [baseX, baseY, baseZ];
    drawCapsule3D(ax3a, p1, p2, cv(7), [0.5 0.5 0.5], 0.12);
end

% 显示全部码垛位碰撞球 (蓝色半透明) — 使用C++实际TCP位置
if ~isempty(profileData)
    % 从profile最后一步获取每个任务的放置TCP位置
    for ti = 1:nTasks
        mask = profileData(:,1)==(ti-1) & profileData(:,2)==5; % task ti-1, seg 5 (place)
        if any(mask)
            rows_ti = profileData(mask,:);
            tcp_mm = rows_ti(end, 14:16);
            % 转换到世界坐标系 (mm→m, 加上base偏移)
            tcp_world = [tcp_mm(1)/1000 + baseX, tcp_mm(2)/1000 + baseY, tcp_mm(3)/1000 + baseZ];
            [Xs,Ys,Zs] = sphere(12);
            r = ENV_COLL.boxR_m;
            surf(ax3a, Xs*r+tcp_world(1), Ys*r+tcp_world(2), Zs*r+tcp_world(3),...
                'FaceColor',[0.2 0.4 0.9],'FaceAlpha',0.15,'EdgeColor','none');
        end
    end
end

drawGround_v11(ax3a, -2.0, 2.0, -3.0, 1.5);
drawCabinet_v11(ax3a, cab, baseX, baseY);
drawFrame_v11(ax3a, frame, CYL_N);
drawPallet_v11(ax3a, pallet, frame, CJK_FONT);

xlabel(ax3a,'X (m)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax3a,'Y (m)','FontSize',10,'FontName',CJK_FONT);
zlabel(ax3a,'Z (m)','FontSize',10,'FontName',CJK_FONT);
title(ax3a,'环境碰撞体: 框架柱(红)+顶梁(黄)+电气柜(橙)+传送带(灰)+放置箱(蓝)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
axis(ax3a,'equal'); grid(ax3a,'on');
xlim(ax3a,[-2.0 2.0]); ylim(ax3a,[-3.0 1.5]); zlim(ax3a,[0 3.5]);
view(ax3a,135,25); camlight('headlight'); lighting(ax3a,'gouraud');

% 3b: .so碰撞几何叠加
ax3b = subplot(1,2,2,'Parent',fig3);
hold(ax3b,'on');
if nTasks > 0
    ti_mid = ceil(nTasks/2);
    q_deg_mid = pall_keyQ(ti_mid,:);
    % SO碰撞体渲染 (真实DH坐标系)
    renderCapsuleRobotHandles(ax3b, q_deg_mid, [baseX,baseY,baseZ], JOINTS);
    
    title(ax3b,sprintf('Task %d: .so 碰撞包络模型 | d=%.0fmm', ti_mid, pall_minD_so(ti_mid)),...
        'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
end
drawGround_v11(ax3b, -2.0, 2.0, -2.0, 1.5);
axis(ax3b,'equal'); grid(ax3b,'on');
view(ax3b,140,30); camlight('headlight'); lighting(ax3b,'gouraud');

sgtitle(fig3,'v15 碰撞几何: 自碰撞体 + 环境碰撞体 + 电气柜 + 传送带',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig3, outputDir, '03_collision_geometry_env');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 4: 码垛 3D 场景 (多视角) + 环境碰撞体                      ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 4: Palletizing 3D scene (%d-box layout)...\n', nBoxes);
tFig = tic;
fig4 = figure('Position',[20 20 1920 1080],'Color','w','Name','Palletizing 3D Scene v15');

% 传送带上箱子位置
convBoxY = zeros(1, nBoxes);
for bi = 1:nBoxes
    convBoxY(bi) = conv.cy + cfg_convBoxYStart + (bi-1)*cfg_convBoxYStep;
end
convBoxX = conv.cx;

% 码垛放置位: 从C++ IK求解TCP位置读取 (消除MATLAB独立计算偏差)
% 注: C++ TCP = 箱子顶面中心; drawBox_v11 需要箱子体积中心 → Z减box.hz/2
placePos = zeros(nBoxes, 3);
placePosFound = false;
for bi = 1:nBoxes
    key = sprintf('box_%d_tcp_mm', bi-1);
    if isfield(pallSummary, key)
        vals = sscanf(pallSummary.(key), '%f %f %f');
        if length(vals)==3
            % C++ TCP在基坐标系(mm) → MATLAB世界坐标系(m), Z减半箱高得到体积中心
            placePos(bi,:) = [vals(1)/1000+baseX, vals(2)/1000+baseY, vals(3)/1000+baseZ - box.hz/2];
            placePosFound = true;
        end
    end
end
if ~placePosFound
    fprintf('  ⚠ C++ box TCP not found in summary, fallback to computed positions\n');
    boxMargin_m = frame.tubeR + box.wy/2 + 0.005;  % 管径+半箱宽+5mm间隙
    boxBackY  = (frame.cy + frame.depthY/2) - boxMargin_m;
    boxFrontY = boxBackY - box.wy - 0.02;
    boxLeftX  = frame.cx - (box.lx/2 + 0.01);
    boxRightX = frame.cx + (box.lx/2 + 0.01);
    % 列优先顺序: 里→外(BK→FR), 左→右(L→R), 下→上(L1→L3)  与C++一致
    for bi = 1:nBoxes
        posGroup = ceil(bi / 3);         % 1..4: BK-L, BK-R, FR-L, FR-R
        layer = mod(bi-1, 3) + 1;       % 1,2,3
        col = floor((posGroup-1) / 2);  % 0=BK, 1=FR
        row = mod(posGroup-1, 2);        % 0=L, 1=R
        if col==0, yy = boxBackY; else, yy = boxFrontY; end
        if row==0, xx = boxLeftX; else, xx = boxRightX; end
        placePos(bi,:) = [xx, yy, palletSurfZ + (layer-0.5)*box.hz];
    end
end
if placePosFound, ppSrc = 'C++ IK'; else, ppSrc = 'computed'; end
fprintf('  placePos: source=%s, range Y=[%.3f,%.3f] Z=[%.3f,%.3f]\n', ...
    ppSrc, min(placePos(:,2)), max(placePos(:,2)), ...
    min(placePos(:,3)), max(placePos(:,3)));

keyPoses = round(linspace(1, nTasks, min(4, nTasks)));
viewAngles = [135 25; 180 30; 90 20; 45 35];

for vi = 1:min(4, length(keyPoses))
    ax = subplot(2,2,vi,'Parent',fig4);
    hold(ax,'on');
    drawGround_v11(ax, -2.0, 2.0, -3.0, 1.5);
    drawCabinet_v11(ax, cab, baseX, baseY);
    drawFrame_v11(ax, frame, CYL_N);
    drawPallet_v11(ax, pallet, frame, CJK_FONT);
    drawConveyor_v15(ax, conv, CYL_N);
    
    % 传送带上的箱子 (仅显示尚未被抓取的箱子: ci > ti)
    bz_center = convSurfZ + box.hz/2;
    ti = keyPoses(vi);
    for ci = 1:nBoxes
        if ci > ti  % 还未被抓取
            drawBox_v11(ax, [convBoxX, convBoxY(ci), bz_center], box);
        end
    end
    % 码垛位上的箱子 (到当前任务为止)
    for ci = 1:min(ti, nBoxes)
        drawBox_v11(ax, placePos(ci,:), box);
    end
    
    % 环境碰撞体叠加 (半透明)
    for eci = 1:size(ENV_COLL.frameColumns, 1)
        fc = ENV_COLL.frameColumns(eci,:);
        p1_w = [fc(1)+baseX, fc(2)+baseY, fc(3)+baseZ];
        p2_w = [fc(1)+baseX, fc(2)+baseY, fc(4)+baseZ];
        drawCapsule3D(ax, p1_w, p2_w, fc(5), [0.9 0.15 0.15], 0.12);
    end
    for eci = 1:size(ENV_COLL.frameTopBars, 1)
        tb = ENV_COLL.frameTopBars(eci,:);
        p1_w = [tb(1)+baseX, tb(2)+baseY, tb(3)+baseZ];
        p2_w = [tb(4)+baseX, tb(5)+baseY, tb(6)+baseZ];
        drawCapsule3D(ax, p1_w, p2_w, tb(7), [0.9 0.7 0.1], 0.10);
    end
    for eci = 1:size(ENV_COLL.cabinet, 1)
        cc = ENV_COLL.cabinet(eci,:);
        p1_w = [cc(1)+baseX, cc(2)+baseY, cc(3)+baseZ];
        p2_w = [cc(4)+baseX, cc(5)+baseY, cc(6)+baseZ];
        drawCapsule3D(ax, p1_w, p2_w, cc(7), [0.8 0.6 0.2], 0.10);
    end
    for eci = 1:size(ENV_COLL.conveyor, 1)
        cv = ENV_COLL.conveyor(eci,:);
        p1_w = [cv(1)+baseX, cv(2)+baseY, cv(3)+baseZ];
        p2_w = [cv(4)+baseX, cv(5)+baseY, cv(6)+baseZ];
        drawCapsule3D(ax, p1_w, p2_w, cv(7), [0.5 0.5 0.5], 0.08);
    end
    
    q_rad = deg2rad(pall_keyQ(ti,:));
    % SO碰撞体渲染 (真实DH坐标系)
    renderCapsuleRobotHandles(ax, pall_keyQ(ti,:), [baseX,baseY,baseZ], JOINTS);
    
    % TCP轨迹 (SO FK2 真实DH坐标, mm→m)
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows = pall_raw(mask,:);
    q_all = rows(:,4:9);
    nR = size(q_all,1); step = max(1,floor(nR/60));
    tcp_traj = zeros(ceil(nR/step),3); idx=0;
    for ri = 1:step:nR
        idx=idx+1;
        if soLoaded
            tcp_traj(idx,:) = soFK2_tcp(q_all(ri,:))/1000 + [baseX, baseY, baseZ];
        else
            skel = fk2Skeleton(q_all(ri,:));
            tcp_traj(idx,:) = skel(5,:) + [baseX, baseY, baseZ];
        end
    end
    tcp_traj = tcp_traj(1:idx,:);
    plot3(ax, tcp_traj(:,1), tcp_traj(:,2), tcp_traj(:,3), '-','Color',[1 0.3 0 0.85],'LineWidth',2.5);
    % TCP起止点标记
    if size(tcp_traj,1)>=2
        plot3(ax, tcp_traj(1,1),tcp_traj(1,2),tcp_traj(1,3),...
            'p','MarkerSize',14,'MarkerFaceColor',[0.1 0.8 0.2],'MarkerEdgeColor','k','LineWidth',1);
        plot3(ax, tcp_traj(end,1),tcp_traj(end,2),tcp_traj(end,3),...
            'h','MarkerSize',12,'MarkerFaceColor',[0.9 0.1 0.6],'MarkerEdgeColor','k','LineWidth',1);
    end
    
    xlabel(ax,'X','FontSize',9,'FontName',CJK_FONT);
    ylabel(ax,'Y','FontSize',9,'FontName',CJK_FONT);
    zlabel(ax,'Z','FontSize',9,'FontName',CJK_FONT);
    title(ax,sprintf('Task %d/%d | d=%.0fmm', ti, nTasks, pall_minD_so(ti)),...
        'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    axis(ax,'equal'); grid(ax,'on');
    xlim(ax,[-2.0 2.0]); ylim(ax,[-3.0 1.5]); zlim(ax,[0 3.5]);
    view(ax, viewAngles(vi,:)); camlight('headlight'); lighting(ax,'gouraud');
end
sgtitle(fig4,sprintf('HR\\_S50-2000 v15 码垛场景 (FIFO %d箱, 环境碰撞体叠加)', nBoxes),...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig4, outputDir, '04_scene_stl_env');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 5: 关节角度/速度/碰撞距离联合分析                          ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 5: Joint + collision analysis...\n');
tFig = tic;
fig5 = figure('Position',[20 20 1920 1080],'Color','w','Name','Joint Analysis v15');
jColors = {[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8],[0.8 0.5 0],[0.5 0 0.8],[0 0.6 0.6]};

task0_mask = pall_raw(:,1)==pall_tasks(1);
task0 = pall_raw(task0_mask,:);
t_task = task0(:,3);

ax5a = subplot(2,3,1,'Parent',fig5); hold(ax5a,'on');
for ji = 1:6, plot(ax5a, t_task, task0(:,3+ji), '-','Color',jColors{ji},'LineWidth',1.5); end
xlabel(ax5a,'Time (s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5a,'Angle (deg)','FontSize',10,'FontName',CJK_FONT);
title(ax5a,'Task 1: 关节角度','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
legend(ax5a,{'J1','J2','J3','J4','J5','J6'},'Location','best','FontSize',8);
grid(ax5a,'on');

ax5b = subplot(2,3,2,'Parent',fig5); hold(ax5b,'on');
for ji = 1:6, plot(ax5b, t_task, task0(:,9+ji), '-','Color',jColors{ji},'LineWidth',1.5); end
xlabel(ax5b,'Time (s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5b,'Velocity (deg/s)','FontSize',10,'FontName',CJK_FONT);
title(ax5b,'Task 1: 关节速度 (S-Curve)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5b,'on');

ax5c = subplot(2,3,3,'Parent',fig5);
hold(ax5c,'on');
yyaxis(ax5c,'left');
plot(ax5c, t_task, so_pall_dist(task0_mask), '-b','LineWidth',1.5);
ylabel(ax5c,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
yyaxis(ax5c,'right');
for ji = [2 3 5]
    plot(ax5c, t_task, task0(:,3+ji), '--','Color',jColors{ji},'LineWidth',1);
end
ylabel(ax5c,'Angle (deg)','FontSize',10,'FontName',CJK_FONT);
xlabel(ax5c,'Time (s)','FontSize',10,'FontName',CJK_FONT);
title(ax5c,'碰撞距离 vs 关节角','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5c,'on');

% 中间任务分析
if nTasks >= 6
    task_mid = 6;
else
    task_mid = nTasks;
end
task_mid_mask = pall_raw(:,1)==pall_tasks(task_mid);
task_mid_data = pall_raw(task_mid_mask,:);
t_mid = task_mid_data(:,3);

ax5d = subplot(2,3,4,'Parent',fig5); hold(ax5d,'on');
for ji=1:6, plot(ax5d, t_mid, task_mid_data(:,3+ji), '-','Color',jColors{ji},'LineWidth',1.5); end
xlabel(ax5d,'Time (s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5d,'Angle (deg)','FontSize',10,'FontName',CJK_FONT);
title(ax5d,sprintf('Task %d: 关节角度', task_mid),'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5d,'on');

ax5e = subplot(2,3,5,'Parent',fig5); hold(ax5e,'on');
for ji=1:6, plot(ax5e, t_mid, task_mid_data(:,9+ji), '-','Color',jColors{ji},'LineWidth',1.5); end
xlabel(ax5e,'Time (s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5e,'Velocity (deg/s)','FontSize',10,'FontName',CJK_FONT);
title(ax5e,sprintf('Task %d: 关节速度', task_mid),'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5e,'on');

% 最后任务碰撞距离
ax5f = subplot(2,3,6,'Parent',fig5);
hold(ax5f,'on');
taskLast_mask = pall_raw(:,1)==pall_tasks(nTasks);
taskLast = pall_raw(taskLast_mask,:);
plot(ax5f, taskLast(:,3), so_pall_dist(taskLast_mask), '-','Color',[0.2 0.5 0.8],'LineWidth',2);
yline(ax5f, 50, 'r--', 'LineWidth',1.5, 'Label', 'Danger 50mm');
xlabel(ax5f,'Time (s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5f,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax5f,sprintf('Task %d: 碰撞距离曲线', nTasks),'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5f,'on');

sgtitle(fig5,sprintf('HR\\_S50-2000 关节+碰撞分析 (v15, %d tasks)', nTasks),...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig5, outputDir, '05_joint_collision_analysis');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 6: 碰撞距离综合分析                                        ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 6: Collision distance analysis...\n');
tFig = tic;
fig6 = figure('Position',[20 20 1800 900],'Color','w','Name','Collision Distance v15');

% 6a: 码垛轨迹碰撞距离
ax6a = subplot(2,2,[1,3],'Parent',fig6);
hold(ax6a,'on');
cmap_task = turbo(nTasks);
for ti = 1:nTasks
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows = pall_raw(mask,:);
    plot(ax6a, rows(:,3), so_pall_dist(mask), '-','LineWidth',1.5,'Color',cmap_task(ti,:));
end
yline(ax6a, 50,'r--','LineWidth',1.5,'Label','Danger 50mm','FontSize',10);
yline(ax6a, 100,'--','Color',[0.8 0.6 0],'LineWidth',1.2,'Label','Caution 100mm');
xlabel(ax6a,'Time (s)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
ylabel(ax6a,'Self-Distance (mm)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
title(ax6a,sprintf('码垛 %d任务: 碰撞距离 vs 时间', nTasks),'FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
legend(ax6a, arrayfun(@(x)sprintf('T%d',x),1:nTasks,'Un',0),'Location','eastoutside','FontSize',7);
grid(ax6a,'on');

ax6b = subplot(2,2,2,'Parent',fig6);
histogram(ax6b, so_pall_dist, 50, 'FaceColor',[0.3 0.6 0.9],'EdgeColor','none','FaceAlpha',0.8);
xline(ax6b, min(so_pall_dist), 'r-', 'LineWidth',2, 'Label',sprintf('Min=%.1fmm',min(so_pall_dist)));
xlabel(ax6b,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax6b,'Count','FontSize',10,'FontName',CJK_FONT);
title(ax6b,'碰撞距离分布','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax6b,'on');

ax6c = subplot(2,2,4,'Parent',fig6);
b = bar(ax6c, pall_minD_so, 'FaceColor','flat');
for ti=1:nTasks
    if pall_minD_so(ti)>400, b.CData(ti,:)=[0.3 0.8 0.4];
    elseif pall_minD_so(ti)>200, b.CData(ti,:)=[1.0 0.8 0.3];
    else, b.CData(ti,:)=[0.9 0.3 0.3]; end
end
set(ax6c,'XTick',1:nTasks,'XTickLabel',arrayfun(@(x)sprintf('T%d',x),1:nTasks,'Un',0));
ylabel(ax6c,'Min Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax6c,'各任务安全裕度','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax6c,'on');

sgtitle(fig6,sprintf('碰撞距离分析 (v15 — %d tasks, FIFO)', nTasks),...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig6, outputDir, '06_collision_distance');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 7: TCP 3D轨迹 + 姿态朝向分析                               ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 7: TCP 3D trajectory + orientation...\n');
tFig = tic;
fig7 = figure('Position',[20 20 1800 900],'Color','w','Name','TCP Trajectory v15');

ax7a = subplot(1,2,1,'Parent',fig7);
hold(ax7a,'on');
cmap_pall = turbo(nTasks);
for ti = 1:nTasks
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows_ti = pall_raw(mask,:);
    q_all_ti = rows_ti(:,4:9);
    nR = size(q_all_ti,1); step = max(1,floor(nR/60));
    tcp_p = zeros(ceil(nR/step),3); idx=0;
    for ri = 1:step:nR
        idx=idx+1;
        if soLoaded
            tcp_p(idx,:) = soFK2_tcp(q_all_ti(ri,:))/1000 + [baseX, baseY, baseZ];
        else
            skel = fk2Skeleton(q_all_ti(ri,:));
            tcp_p(idx,:) = skel(5,:) + [baseX, baseY, baseZ];
        end
    end
    tcp_p = tcp_p(1:idx,:);
    plot3(ax7a, tcp_p(:,1), tcp_p(:,2), tcp_p(:,3), '-','LineWidth',1.5,'Color',[cmap_pall(ti,:) 0.8]);
    % TCP起止点标记
    if size(tcp_p,1)>=2
        plot3(ax7a, tcp_p(1,1),tcp_p(1,2),tcp_p(1,3),...
            'p','MarkerSize',12,'MarkerFaceColor',[0.1 0.8 0.2],'MarkerEdgeColor','k','LineWidth',1);
        plot3(ax7a, tcp_p(end,1),tcp_p(end,2),tcp_p(end,3),...
            'h','MarkerSize',10,'MarkerFaceColor',[0.9 0.1 0.6],'MarkerEdgeColor','k','LineWidth',1);
    end
end
drawGround_v11(ax7a, -2.0, 2.0, -3.0, 1.5);
q_home = deg2rad([0,-90,0,0,90,0]);
    % SO碰撞体渲染 (真实DH坐标系)
    renderCapsuleRobotHandles(ax7a, [0,-90,0,0,90,0], [baseX,baseY,baseZ], JOINTS);

% 环境碰撞体
for eci = 1:size(ENV_COLL.frameColumns, 1)
    fc = ENV_COLL.frameColumns(eci,:);
    p1_w = [fc(1)+baseX, fc(2)+baseY, fc(3)+baseZ];
    p2_w = [fc(1)+baseX, fc(2)+baseY, fc(4)+baseZ];
    drawCapsule3D(ax7a, p1_w, p2_w, fc(5), [0.9 0.15 0.15], 0.10);
end
for eci = 1:size(ENV_COLL.frameTopBars, 1)
    tb = ENV_COLL.frameTopBars(eci,:);
    p1_w = [tb(1)+baseX, tb(2)+baseY, tb(3)+baseZ];
    p2_w = [tb(4)+baseX, tb(5)+baseY, tb(6)+baseZ];
    drawCapsule3D(ax7a, p1_w, p2_w, tb(7), [0.9 0.7 0.1], 0.08);
end
for eci = 1:size(ENV_COLL.cabinet, 1)
    cc = ENV_COLL.cabinet(eci,:);
    p1_w = [cc(1)+baseX, cc(2)+baseY, cc(3)+baseZ];
    p2_w = [cc(4)+baseX, cc(5)+baseY, cc(6)+baseZ];
    drawCapsule3D(ax7a, p1_w, p2_w, cc(7), [0.8 0.6 0.2], 0.08);
end
for eci = 1:size(ENV_COLL.conveyor, 1)
    cv = ENV_COLL.conveyor(eci,:);
    p1_w = [cv(1)+baseX, cv(2)+baseY, cv(3)+baseZ];
    p2_w = [cv(4)+baseX, cv(5)+baseY, cv(6)+baseZ];
    drawCapsule3D(ax7a, p1_w, p2_w, cv(7), [0.5 0.5 0.5], 0.06);
end

xlabel(ax7a,'X','FontSize',10,'FontName',CJK_FONT);
ylabel(ax7a,'Y','FontSize',10,'FontName',CJK_FONT);
zlabel(ax7a,'Z','FontSize',10,'FontName',CJK_FONT);
title(ax7a,sprintf('码垛 TCP 3D路径 (%d tasks)', nTasks),'FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
grid(ax7a,'on'); axis(ax7a,'equal'); view(ax7a,140,30);
camlight('headlight'); lighting(ax7a,'gouraud');

ax7b = subplot(1,2,2,'Parent',fig7);
hold(ax7b,'on');
if soLoaded && ~isempty(tcpOrientError_deg)
    stride = max(1, floor(nPall/200));
    for ri = 1:stride:nPall
        if soLoaded
            pos = soFK2_tcp(pall_raw(ri,4:9))/1000 + [baseX, baseY, baseZ];
        else
            skel = fk2Skeleton(pall_raw(ri,4:9));
            pos = skel(5,:) + [baseX, baseY, baseZ];
        end
        % 使用FK2 A,B,C 导出的TCP Z轴 (so_pall_tcpAxis), 避免骨架近似误差
        zdir = so_pall_tcpAxis(ri,:) * 0.05;  % 已归一化, 缩放用于显示
        err = tcpOrientError_deg(ri);
        if err < 15, c = [0 0.7 0.2];
        elseif err < 30, c = [0.8 0.6 0];
        else, c = [0.9 0.1 0.1]; end
        quiver3(ax7b, pos(1),pos(2),pos(3), zdir(1),zdir(2),zdir(3),...
            'Color',c,'LineWidth',0.8,'MaxHeadSize',0.3,'AutoScale','off');
    end
end
drawGround_v11(ax7b, -2.0, 2.0, -3.0, 1.5);
    % SO碰撞体渲染 (真实DH坐标系)
    renderCapsuleRobotHandles(ax7b, [0,-90,0,0,90,0], [baseX,baseY,baseZ], JOINTS);
xlabel(ax7b,'X','FontSize',10,'FontName',CJK_FONT);
ylabel(ax7b,'Y','FontSize',10,'FontName',CJK_FONT);
zlabel(ax7b,'Z','FontSize',10,'FontName',CJK_FONT);
title(ax7b,'TCP Z轴朝向 (绿=好 黄=警 红=差)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
grid(ax7b,'on'); axis(ax7b,'equal'); view(ax7b,140,30);
camlight('headlight'); lighting(ax7b,'gouraud');

sgtitle(fig7,sprintf('TCP轨迹+姿态分析 (v15 — %d箱FIFO码垛)', nBoxes),...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig7, outputDir, '07_tcp_trajectory_orientation');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 8: 码垛3D动态回放 (低模STL + 环境碰撞体)                   ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 8: Dynamic replay (%d-box, env collision overlay)...\n', nBoxes);
tFig = tic;
fig8 = figure('Position',[50 50 1700 950],'Color','w','Renderer','opengl','Name','Dynamic Replay v15');

ax3d = axes('Parent',fig8,'Position',[0.02 0.05 0.64 0.88]);
hold(ax3d,'on');
drawGround_v11(ax3d, -2.0, 2.0, -3.0, 1.5);
drawCabinet_v11(ax3d, cab, baseX, baseY);
drawFrame_v11(ax3d, frame, CYL_N);
drawPallet_v11(ax3d, pallet, frame, CJK_FONT);
drawConveyor_v15(ax3d, conv, CYL_N);

% 环境碰撞体: 框架柱 (红色半透明, 始终显示)
for eci = 1:size(ENV_COLL.frameColumns, 1)
    fc = ENV_COLL.frameColumns(eci,:);
    p1_w = [fc(1)+baseX, fc(2)+baseY, fc(3)+baseZ];
    p2_w = [fc(1)+baseX, fc(2)+baseY, fc(4)+baseZ];
    drawCapsule3D(ax3d, p1_w, p2_w, fc(5), [0.9 0.15 0.15], 0.10);
end
% 环境碰撞体: 框架顶梁 (黄色半透明)
for eci = 1:size(ENV_COLL.frameTopBars, 1)
    tb = ENV_COLL.frameTopBars(eci,:);
    p1_w = [tb(1)+baseX, tb(2)+baseY, tb(3)+baseZ];
    p2_w = [tb(4)+baseX, tb(5)+baseY, tb(6)+baseZ];
    drawCapsule3D(ax3d, p1_w, p2_w, tb(7), [0.9 0.7 0.1], 0.10);
end
% 环境碰撞体: 电气柜 (橙色半透明)
for eci = 1:size(ENV_COLL.cabinet, 1)
    cc = ENV_COLL.cabinet(eci,:);
    p1_w = [cc(1)+baseX, cc(2)+baseY, cc(3)+baseZ];
    p2_w = [cc(4)+baseX, cc(5)+baseY, cc(6)+baseZ];
    drawCapsule3D(ax3d, p1_w, p2_w, cc(7), [0.8 0.6 0.2], 0.08);
end
% 环境碰撞体: 传送带 (灰色半透明)
for eci = 1:size(ENV_COLL.conveyor, 1)
    cv = ENV_COLL.conveyor(eci,:);
    p1_w = [cv(1)+baseX, cv(2)+baseY, cv(3)+baseZ];
    p2_w = [cv(4)+baseX, cv(5)+baseY, cv(6)+baseZ];
    drawCapsule3D(ax3d, p1_w, p2_w, cv(7), [0.5 0.5 0.5], 0.06);
end

bz_center = convSurfZ + box.hz/2;
hConvBoxes = gobjects(nBoxes,1); hConvLabels = gobjects(nBoxes,1);
for ci = 1:nBoxes
    hConvBoxes(ci) = drawBox_v11(ax3d, [convBoxX, convBoxY(ci), bz_center], box);
    hConvLabels(ci) = text(ax3d, convBoxX, convBoxY(ci), convSurfZ+box.hz+0.08,...
        sprintf('#%d',ci),'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
        'HorizontalAlignment','center','Color',[.8 .3 0]);
end
hPlacedBoxes = gobjects(nBoxes,1); hPlacedLabels = gobjects(nBoxes,1);
hPlacedCollSpheres = gobjects(nBoxes,1);  % 碰撞球可视化
placedFlag = false(nBoxes,1);

xlabel(ax3d,'X (m)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
ylabel(ax3d,'Y (m)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
zlabel(ax3d,'Z (m)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
axis(ax3d,'equal'); grid(ax3d,'on');
xlim(ax3d,[-2.0 2.0]); ylim(ax3d,[-3.0 1.5]); zlim(ax3d,[0 3.5]);
view(ax3d,VIEW_AZ,VIEW_EL); camlight('headlight'); lighting(ax3d,'gouraud');
hTitle = title(ax3d,'Loading...','FontSize',16,'FontWeight','bold','FontName',CJK_FONT);

ax_info = axes('Parent',fig8,'Position',[0.68 0.05 0.30 0.88]);
axis(ax_info,'off'); xlim(ax_info,[0 1]); ylim(ax_info,[0 1]); hold(ax_info,'on');

prevRobotH = gobjects(0);
hCarryBox = gobjects(0);
hToolCollSphere = gobjects(0);  % 工具碰撞球
hTrail = gobjects(0);
hGlobalTrail = gobjects(0);    % 全局TCP轨迹 (跨任务)
globalTrail = [];               % 全局TCP点积累
hTcpStartEnd = gobjects(0);    % 任务起止点标记
allGifFrames = {};
boxForTask = min((1:nTasks)', nBoxes);

% 动画任务限制 (调试用: 仅播放前N个任务)
if cfg_animTaskLimit > 0
    nAnimTasks = min(cfg_animTaskLimit, nTasks);
else
    nAnimTasks = nTasks;
end
fprintf('  动画: %d/%d tasks, subsample=%d, %d boxes\n', nAnimTasks, nTasks, ANIM_SUBSAMPLE, nBoxes);

for ti = 1:nAnimTasks
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows = pall_raw(mask,:);
    nR = size(rows,1);
    taskTrail = [];
    bi = boxForTask(ti);
    prevSeg = -1;         % 段号变化检测
    soToolActive = false;  % SO库工具碰撞球状态
    
    % 清除上一任务的起止点标记
    if ~isempty(hTcpStartEnd) && any(isvalid(hTcpStartEnd))
        delete(hTcpStartEnd(isvalid(hTcpStartEnd)));
    end
    hTcpStartEnd = gobjects(0);
    
    for ri = 1:ANIM_SUBSAMPLE:nR
        q_deg = rows(ri, 4:9);
        q_rad = deg2rad(q_deg);
        vel = rows(ri, 10:15);
        seg = rows(ri, 2);
        
        % SO库工具碰撞体管理 (匹配C++ seg=2启用/seg=6移除)
        if soLoaded && seg ~= prevSeg
            % 进入seg 2: 启用工具碰撞球 (吸附箱子, toolIdx=6)
            if seg == 2 && ~soToolActive && bi <= nBoxes
                toolOffset_mm = [0, 0, -box.hz/2*1000];  % 箱子中心在TCP下方
                calllib('libHRCInterface', 'setCPToolCollisionBallShapeInterface', ...
                    int64(6), toolOffset_mm, 225.0);  % r=225mm
                soToolActive = true;
            end
            % 进入seg 6: 移除工具碰撞球, 注册已放置箱子为环境障碍
            if seg == 6 && soToolActive && bi <= nBoxes
                calllib('libHRCInterface', 'removeCPToolCollisonInterface', int64(6));
                soToolActive = false;
                boxCenter_mm = placePos(bi,:) * 1000;  % m→mm
                calllib('libHRCInterface', 'addEnvObstacleBallInterface', ...
                    int64(45 + bi), boxCenter_mm, 250.0);  % envId=46..57, r=250mm
            end
            prevSeg = seg;
        end
        
        if soLoaded
            % 碰撞检测 (update已在renderCapsuleRobotHandles中执行, 但此处需先做)
            velArr = vel; accArr = zeros(1,6);
            calllib('libHRCInterface','updateACAreaConstrainPackageInterface',q_deg,velArr,accArr);
            pairArr = int64([0,0]); distVal = 0.0;
            [~,pairArr,distVal] = calllib('libHRCInterface','checkCPSelfCollisionInterface',pairArr,distVal);
            dist = distVal;
            timing.collisionCalls = timing.collisionCalls + 1;
        else
            dist = rows(ri, 16);
        end
        
        carrying = (seg >= 2 && seg <= 5) && (bi <= nBoxes);
        
        if ~isempty(prevRobotH) && any(isvalid(prevRobotH))
            delete(prevRobotH(isvalid(prevRobotH)));
        end
        
        % SO碰撞体渲染+TCP (真实DH坐标系, getUIInfoMation)
        [prevRobotH, tcp] = renderCapsuleRobotHandles(ax3d, q_deg, [baseX,baseY,baseZ], JOINTS);
        taskTrail = [taskTrail; tcp]; %#ok<AGROW>
        
        % 携带箱子 + 工具碰撞球
        if ~isempty(hCarryBox) && any(isvalid(hCarryBox))
            delete(hCarryBox(isvalid(hCarryBox)));
        end
        hCarryBox = gobjects(0);
        if ~isempty(hToolCollSphere) && any(isvalid(hToolCollSphere))
            delete(hToolCollSphere(isvalid(hToolCollSphere)));
        end
        hToolCollSphere = gobjects(0);
        
        if carrying
            hCarryBox = drawBox_v11(ax3d, [tcp(1),tcp(2),tcp(3)-0.01], box);
            % 工具碰撞球 (半透明绿色)
            toolR = 0.225;  % 225mm → m
            [Xs,Ys,Zs] = sphere(10);
            hToolCollSphere = surf(ax3d, Xs*toolR+tcp(1), Ys*toolR+tcp(2), Zs*toolR+tcp(3)-0.125,...
                'FaceColor',[0.2 0.8 0.3],'FaceAlpha',0.08,'EdgeColor','none');
        end
        
        % 传送带箱子可见性
        for ci2 = 1:nBoxes
            vis = 'on';
            if ci2 < bi || (ci2 == bi && carrying), vis = 'off'; end
            if isvalid(hConvBoxes(ci2)), set(hConvBoxes(ci2),'Visible',vis); end
            if isvalid(hConvLabels(ci2)), set(hConvLabels(ci2),'Visible',vis); end
        end
        if seg >= 2 && bi <= nBoxes
            if isvalid(hConvBoxes(bi)), set(hConvBoxes(bi),'Visible','off'); end
            if isvalid(hConvLabels(bi)), set(hConvLabels(bi),'Visible','off'); end
        end
        
        % 放置箱子 + 碰撞球
        if seg >= 6 && bi <= nBoxes && ~placedFlag(bi)
            placedFlag(bi) = true;
            hPlacedBoxes(bi) = drawBox_v11(ax3d, placePos(bi,:), box);
            hPlacedLabels(bi) = text(ax3d, placePos(bi,1),placePos(bi,2),...
                placePos(bi,3)+box.hz/2+0.06,sprintf('#%d',bi),...
                'FontSize',9,'FontWeight','bold','FontName',CJK_FONT,...
                'HorizontalAlignment','center','Color',[0 0.4 0.7]);
            % 放置碰撞球 (蓝色半透明)
            r_box = ENV_COLL.boxR_m;
            [Xs,Ys,Zs] = sphere(10);
            hPlacedCollSpheres(bi) = surf(ax3d, Xs*r_box+placePos(bi,1),...
                Ys*r_box+placePos(bi,2), Zs*r_box+placePos(bi,3),...
                'FaceColor',[0.2 0.4 0.9],'FaceAlpha',0.08,'EdgeColor','none');
        end
        
        % --- TCP轨迹显示 ---
        % 全局轨迹 (灰色, 所有已完成任务)
        globalTrail = [globalTrail; tcp]; %#ok<AGROW>
        if ~isempty(hGlobalTrail) && any(isvalid(hGlobalTrail))
            delete(hGlobalTrail(isvalid(hGlobalTrail)));
        end
        hGlobalTrail = gobjects(0);
        if size(globalTrail,1)>1
            hGlobalTrail = plot3(ax3d, globalTrail(:,1),globalTrail(:,2),globalTrail(:,3),...
                '-','Color',[0.5 0.5 0.5 0.4],'LineWidth',1.2);
        end
        
        % 当前任务轨迹 (橙色高亮)
        if ~isempty(hTrail) && any(isvalid(hTrail))
            delete(hTrail(isvalid(hTrail)));
        end
        hTrail = gobjects(0);
        if size(taskTrail,1)>1
            hTrail = plot3(ax3d, taskTrail(:,1),taskTrail(:,2),taskTrail(:,3),...
                '-','Color',[1 0.3 0 0.85],'LineWidth',2.5);
        end
        
        % 起止点标记: 绿色=起点, 品红=终点(实时更新)
        if ~isempty(hTcpStartEnd) && any(isvalid(hTcpStartEnd))
            delete(hTcpStartEnd(isvalid(hTcpStartEnd)));
        end
        hTcpStartEnd = gobjects(0);
        if size(taskTrail,1)>=1
            hTcpStartEnd(1) = plot3(ax3d, taskTrail(1,1),taskTrail(1,2),taskTrail(1,3),...
                'p','MarkerSize',16,'MarkerFaceColor',[0.1 0.8 0.2],'MarkerEdgeColor','k','LineWidth',1);
            hTcpStartEnd(2) = plot3(ax3d, tcp(1),tcp(2),tcp(3),...
                'h','MarkerSize',14,'MarkerFaceColor',[0.9 0.1 0.6],'MarkerEdgeColor','k','LineWidth',1);
        end
        
        if dist>400, dColor=[0 0.7 0.2]; elseif dist>200, dColor=[0.8 0.6 0]; else, dColor=[0.9 0.1 0.1]; end
        set(hTitle,'String',sprintf('v15 | Task %d/%d Seg%d | self=%.0fmm env=%d',ti,nTasks,seg,dist,envCollisions));
        
        cla(ax_info);
        drawInfoPanel_v15(ax_info, ti, nTasks, q_deg, vel, tcp, dist, rows(ri,3), ...
            CJK_FONT, dColor, soLoaded, carrying, sum(placedFlag), nBoxes, envCollisions);
        
        drawnow limitrate;
        
        if isHeadless && mod(ri, GIF_SUBSAMPLE)==1
            allGifFrames{end+1} = getframe(fig8); %#ok<AGROW>
        end
    end
    
    if ~isempty(hCarryBox) && any(isvalid(hCarryBox)), delete(hCarryBox(isvalid(hCarryBox))); end
    hCarryBox = gobjects(0);
    if ~isempty(hToolCollSphere) && any(isvalid(hToolCollSphere)), delete(hToolCollSphere(isvalid(hToolCollSphere))); end
    hToolCollSphere = gobjects(0);
    
    % 确保SO工具球在任务结束后被移除 (防止数据不完整时遗留)
    if soLoaded && soToolActive
        calllib('libHRCInterface', 'removeCPToolCollisonInterface', int64(6));
        soToolActive = false;
        fprintf('    [WARN] Task %d 结束但工具球未自动移除, 已强制清理\n', ti);
    end
    
    fprintf('  Task %d/%d (%d frames) — placed: %d/%d\n', ti, nTasks, ceil(nR/ANIM_SUBSAMPLE), sum(placedFlag), nBoxes);
end

% 动画结束后补全未标记的放置箱子 (最后一个任务=回原位, seg从未达到6)
for bi = 1:nBoxes
    if ~placedFlag(bi)
        placedFlag(bi) = true;
        hPlacedBoxes(bi) = drawBox_v11(ax3d, placePos(bi,:), box);
        hPlacedLabels(bi) = text(ax3d, placePos(bi,1),placePos(bi,2),...
            placePos(bi,3)+box.hz/2+0.06,sprintf('#%d',bi),...
            'FontSize',9,'FontWeight','bold','FontName',CJK_FONT,...
            'HorizontalAlignment','center','Color',[0 0.4 0.7]);
        r_box = ENV_COLL.boxR_m;
        [Xs,Ys,Zs] = sphere(10);
        hPlacedCollSpheres(bi) = surf(ax3d, Xs*r_box+placePos(bi,1),...
            Ys*r_box+placePos(bi,2), Zs*r_box+placePos(bi,3),...
            'FaceColor',[0.2 0.4 0.9],'FaceAlpha',0.08,'EdgeColor','none');
        fprintf('  补全放置箱子 #%d\n', bi);
    end
end

saveFig(fig8, outputDir, '08_dynamic_replay_v15');
if isHeadless && ~isempty(allGifFrames)
    gifFile = fullfile(outputDir, 'palletizing_v15.gif');
    for gi = 1:length(allGifFrames)
        [A,map] = rgb2ind(allGifFrames{gi}.cdata, 128);
        if gi==1, imwrite(A,map,gifFile,'gif','LoopCount',0,'DelayTime',0.12);
        else, imwrite(A,map,gifFile,'gif','WriteMode','append','DelayTime',0.12); end
    end
    fprintf('  GIF: %s (%d frames)\n', gifFile, length(allGifFrames));
end
timing.animation_ms = toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 9: 综合仪表盘 + v5.0统计                                   ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 9: Summary dashboard (v5.0 stats)...\n');
tFig = tic;
fig9 = figure('Position',[20 20 1920 1080],'Color','w','Name','Dashboard v15');

% 9a: 安全评分
ax9a = subplot(2,3,1,'Parent',fig9);
scores = [min(so_pall_dist)/700*100, ...
          100*(envCollisions==0), ...  % 环境碰撞评分
          min(100, mean(pall_minD_so)/5), ...
          95, ...  % S-Curve质量
          100];
if ~isempty(tcpOrientError_deg)
    scores(end) = 100 - mean(tcpOrientError_deg)/1.8;
end
scores = max(0, min(scores, 100));
cats = {'Self\nSafety','Env\nSafety','Avg\nMargin','S-Curve','TCP\nOrient'};
b9 = bar(ax9a, scores, 'FaceColor','flat');
for si = 1:length(scores)
    if scores(si)>=80, b9.CData(si,:)=[0.3 0.8 0.4];
    elseif scores(si)>=50, b9.CData(si,:)=[1.0 0.8 0.3];
    else, b9.CData(si,:)=[0.9 0.3 0.3]; end
end
set(ax9a,'XTickLabel',cats,'FontSize',8,'FontName',CJK_FONT);
ylabel(ax9a,'Score (%)','FontSize',10,'FontName',CJK_FONT);
title(ax9a,'安全+质量评分','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
ylim(ax9a,[0 105]); grid(ax9a,'on');

% 9b: 每任务碰撞
ax9b = subplot(2,3,2,'Parent',fig9);
bar(ax9b, pall_minD_so, 'FaceColor',[0.3 0.7 0.5]);
set(ax9b,'XTick',1:nTasks,'XTickLabel',arrayfun(@(x)sprintf('T%d',x),1:nTasks,'Un',0));
ylabel(ax9b,'Min Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax9b,sprintf('码垛安全裕度 (%d tasks)', nTasks),'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax9b,'on');

% 9c: 耗时分解
ax9c = subplot(2,3,3,'Parent',fig9);
timings_arr = [timing.soInit_ms, timing.meshLoad_ms, timing.dataLoad_ms, ...
               timing.collisionCheck_ms, timing.rendering_ms, timing.animation_ms];
labels_arr = {'碰撞初始化','STL加载','数据加载','碰撞检测','静态渲染','动画'};
valid = timings_arr > 0;
if any(valid)
    pie(ax9c, timings_arr(valid), labels_arr(valid));
    title(ax9c,'耗时分解','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
end

% 9d: 性能Profile
ax9d = subplot(2,3,4,'Parent',fig9); axis(ax9d,'off');
xlim(ax9d,[0 1]); ylim(ax9d,[0 1]); hold(ax9d,'on');
yP = 0.95;
text(ax9d,0.05,yP,'MATLAB Performance Profiling','FontSize',14,'FontWeight','bold',...
    'FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
yP = yP-0.06;
stats = {
    sprintf('碰撞.so初始化:      %8.2f ms', timing.soInit_ms);
    sprintf('STL网格加载:         %8.2f ms', timing.meshLoad_ms);
    sprintf('C++数据加载:         %8.2f ms', timing.dataLoad_ms);
    sprintf('碰撞检测总计:        %8.2f ms (%d calls)', timing.collisionCheck_ms, timing.collisionCalls);
    sprintf('  平均/call:         %8.2f us', timing.collisionCheck_ms/max(1,timing.collisionCalls)*1000);
    sprintf('FK调用:              %8d calls', timing.fkCalls);
    sprintf('静态图渲染:          %8.2f ms', timing.rendering_ms);
    sprintf('动画渲染:            %8.2f ms', timing.animation_ms);
};
for si = 1:length(stats)
    text(ax9d,0.05,yP,stats{si},'FontSize',10,'FontName',CJK_FONT,'Color',[0.2 0.2 0.2]);
    yP=yP-0.04;
end

% 9e: C++ v5.0 Pipeline Stats
ax9e = subplot(2,3,5,'Parent',fig9); axis(ax9e,'off');
xlim(ax9e,[0 1]); ylim(ax9e,[0 1]); hold(ax9e,'on');
yP = 0.95;
text(ax9e,0.05,yP,'C++ v5.0 Pipeline Stats','FontSize',14,'FontWeight','bold',...
    'FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
yP = yP-0.06;
pipeStats = {
    sprintf('码垛位: %s, 运动段: %s', getField(pallSummary,'positions','?'), getField(pallSummary,'segments','?'));
    sprintf('运动时间: %s s', getField(pallSummary,'total_motion_s','?'));
    sprintf('自碰撞: %d | 环境碰撞: %d', selfCollisions, envCollisions);
    sprintf('最小距离: %s mm', getField(pallSummary,'min_dist_mm','?'));
    sprintf('任务顺序: %s', taskOrder);
    '';
    sprintf('环境障碍: %s', envObstacles);
    sprintf('工具碰撞体: %s', toolCollision);
    '';
    sprintf('RRT*规划: %s ms', getField(pallSummary,'planning_total_ms','?'));
    sprintf('S曲线参数化: %s ms', getField(pallSummary,'param_total_ms','?'));
    sprintf('碰撞运行时: %s ms', getField(pallSummary,'collision_runtime_ms','?'));
    sprintf('总耗时: %s s', getField(pallSummary,'elapsed_s','?'));
};
for si = 1:length(pipeStats)
    if isempty(pipeStats{si}), yP=yP-0.02; continue; end
    col = [0.2 0.2 0.2];
    if contains(pipeStats{si},'环境碰撞: 0'), col = [0 0.6 0.2]; end
    text(ax9e,0.05,yP,pipeStats{si},'FontSize',10,'FontName',CJK_FONT,'Color',col);
    yP=yP-0.04;
end

% 9f: STL + 碰撞信息
ax9f = subplot(2,3,6,'Parent',fig9); axis(ax9f,'off');
xlim(ax9f,[0 1]); ylim(ax9f,[0 1]); hold(ax9f,'on');
yP = 0.95;
text(ax9f,0.05,yP,'System Info','FontSize',14,'FontWeight','bold',...
    'FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
yP = yP-0.06;
sysInfo = {
    sprintf('STL原始: %d面, 低模: %d面 (%.0f%%↓)', totalFaces, totalFacesLow, (1-totalFacesLow/totalFaces)*100);
    sprintf('碰撞源: %s', ifelse(soLoaded, '.so (实时)', 'C++ (离线)'));
    sprintf('箱子: %d个 (%.0f×%.0f×%.0fcm)', nBoxes, box.lx*100, box.wy*100, box.hz*100);
    sprintf('码垛布局: 3层×2行×2列');
    '';
    sprintf('框架碰撞柱: 4根 (r=50mm, 胶囊体)');
    sprintf('箱碰撞球: r=250mm (已放置)');
    sprintf('工具碰撞球: r=225mm (搬运中)');
    '';
    sprintf('.so avg: %s us/call', getField(pallSummary,'coll_total_avg_us','?'));
    sprintf('.so calls: %s', getField(pallSummary,'coll_calls','?'));
};
if ~isempty(tcpOrientError_deg)
    sysInfo{end+1} = sprintf('TCP姿态: mean=%.1f° max=%.1f°', mean(tcpOrientError_deg), max(tcpOrientError_deg));
end
for si = 1:length(sysInfo)
    if isempty(sysInfo{si}), yP=yP-0.02; continue; end
    text(ax9f,0.05,yP,sysInfo{si},'FontSize',10,'FontName',CJK_FONT,'Color',[0.2 0.2 0.2]);
    yP=yP-0.04;
end

sgtitle(fig9,sprintf('HR\\_S50-2000 v15 仪表盘 (v5.0 环境碰撞 + FIFO + %d箱)', nBoxes),...
    'FontSize',15,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig9, outputDir, '09_dashboard_v15');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                         完成 + 性能报告                             ║
%% ╚══════════════════════════════════════════════════════════════════════╝
totalElapsed_s = toc(tTotal);
timing.totalFigures_ms = toc(startTime)*1000;

fprintf('\n');
fprintf([char(9556) repmat(char(9552),1,72) char(9559) '\n']);
fprintf([char(9553) '  v15.0 全链路性能分析报告                                            ' char(9553) '\n']);
fprintf([char(9562) repmat(char(9552),1,72) char(9565) '\n']);
fprintf('\n');
fprintf('  ╔════════════════════════════════════════════════════════════╗\n');
fprintf('  ║  阶段               耗时(ms)    耗时(s)  占比           ║\n');
fprintf('  ╠════════════════════════════════════════════════════════════╣\n');
totalMs = totalElapsed_s * 1000;
phases = {'碰撞.so初始化', timing.soInit_ms;
          'STL网格加载',   timing.meshLoad_ms;
          'C++数据加载',   timing.dataLoad_ms;
          '碰撞检测(.so)', timing.collisionCheck_ms;
          '静态图渲染',    timing.rendering_ms;
          '动画渲染',      timing.animation_ms};
for ph = 1:size(phases,1)
    fprintf('  ║  %-18s %8.1f  %7.2f  %5.1f%%     ║\n', ...
        phases{ph,1}, phases{ph,2}, phases{ph,2}/1000, phases{ph,2}/totalMs*100);
end
otherMs = totalMs - timing.soInit_ms - timing.meshLoad_ms - timing.dataLoad_ms ...
    - timing.collisionCheck_ms - timing.rendering_ms - timing.animation_ms;
fprintf('  ║  %-18s %8.1f  %7.2f  %5.1f%%     ║\n', '其他(FK/IO/GC)', otherMs, otherMs/1000, otherMs/totalMs*100);
fprintf('  ╠════════════════════════════════════════════════════════════╣\n');
fprintf('  ║  总计               %8.1f  %7.2f  100.0%%     ║\n', totalMs, totalElapsed_s);
fprintf('  ╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');
fprintf('  v5.0 碰撞统计:\n');
fprintf('    自碰撞: %d | 环境碰撞: %d\n', selfCollisions, envCollisions);
fprintf('    任务顺序: %s\n', taskOrder);
fprintf('    环境障碍: %s\n', envObstacles);
fprintf('    工具碰撞: %s\n', toolCollision);
fprintf('\n');
fprintf('  MATLAB碰撞调用:\n');
fprintf('    总调用: %d | 平均: %.2f us/call\n', timing.collisionCalls, ...
    timing.collisionCheck_ms / max(1, timing.collisionCalls) * 1000);
fprintf('    FK调用: %d\n', timing.fkCalls);
fprintf('\n');
fprintf('  优化效果:\n');
fprintf('    STL降面: %d → %d (%.0f%% 减少)\n', totalFaces, totalFacesLow, (1-totalFacesLow/totalFaces)*100);
fprintf('    碰撞源: %s\n', ifelse(soLoaded, 'libHRCInterface.so (实时)', 'C++预计算'));
if ~isempty(tcpOrientError_deg)
    fprintf('    TCP姿态: mean=%.1f° max=%.1f° within±%.0f°=%.1f%%\n', ...
        mean(tcpOrientError_deg), max(tcpOrientError_deg), TCP_ORIENT_TOL_DEG, ...
        sum(tcpOrientError_deg<=TCP_ORIENT_TOL_DEG)/nPall*100);
end
fprintf('\n');

fList = dir(fullfile(outputDir, '*.png'));
fprintf('  生成文件:\n');
for i=1:length(fList), fprintf('    %s\n', fList(i).name); end
fList = dir(fullfile(outputDir, '*.gif'));
for i=1:length(fList), fprintf('    %s (GIF)\n', fList(i).name); end

if soLoaded && libisloaded('libHRCInterface')
    unloadlibrary('libHRCInterface');
    fprintf('\n  libHRCInterface.so 已卸载\n');
end

if isHeadless, close all; end
fprintf('\nv15.0 complete! (%.1f s)\n', totalElapsed_s);
end % testS50_Palletizing_v15


%% ═══════════════════════════════════════════════════════════════════════
%%                         辅助函数
%% ═══════════════════════════════════════════════════════════════════════

function result = ifelse(cond, trueVal, falseVal)
    if cond, result = trueVal; else, result = falseVal; end
end

function T_all = urdfFK(JOINTS, q_rad)
    % 返回8个变换矩阵: T{1}=base(eye4), T{2..7}=链路1..6, T{8}=TCP末端
    % elfin_end_joint: xyz=[0,0,0.1345] rpy=[0,0,pi] (固定关节, 无旋转自由度)
    T_all = cell(8,1); T_all{1} = eye(4);
    for i = 1:6
        xyz = JOINTS(i,1:3); rpy = JOINTS(i,4:6);
        T_all{i+1} = T_all{i} * makeTrans(xyz) * makeRotRPY(rpy) * makeRotZ(q_rad(i));
    end
    % 添加末端关节: elfin_end_joint (固定偏移 134.5mm + 绕Z旋转180°)
    T_all{8} = T_all{7} * makeTrans([0, 0, 0.1345]) * makeRotRPY([0, 0, pi]);
end

function joints_m = fk2Skeleton(q_deg)
    % [已废弃] 旧版 FK2 经验骨架模型 — 坐标系与 v1.0.0 FK2 不一致
    % 仅保留作为 SO 库不可用时的后备
    % v1.0.0 FK2 真实值:
    %   ZERO [0,0,0,0,0,0]   → TCP=(-1841.5, -390.2, 138.0) mm
    %   HOME [0,-90,0,0,90,0]→ TCP=(-158.5, -255.7, 2272.5) mm
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

function tcp_mm = soFK2_tcp(q_deg)
    % 调用 SO 库 FK2 获取 TCP 位置 (mm, 真实DH坐标系)
    % 输入: q_deg — 关节角 (deg)
    % 输出: tcp_mm — [X, Y, Z] (mm)
    % 注意: 使用 q_deg 的副本, FK2 不修改输入
    q_fk = double(q_deg(:)');
    tcpS = libstruct('MC_COORD_REF');
    tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
    [~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_fk, tcpS);
    tcp_mm = [tcpS.X, tcpS.Y, tcpS.Z];
end

function [bodyData, tcp_mm] = soGetCollisionBodies(q_deg)
    % 调用 SO 库 update + getUIInfo + FK2, 返回碰撞体数据 (mm)
    % bodyData.type(i) : 1=球, 2=胶囊
    % bodyData.data(i,:) : 9-element, 胶囊=[p1x,p1y,p1z, p2x,p2y,p2z, ...]
    % bodyData.radius(i) : 半径 (mm)
    % tcp_mm : FK2 TCP [X,Y,Z] (mm)
    q_up = double(q_deg(:)');
    calllib('libHRCInterface','updateACAreaConstrainPackageInterface', q_up, zeros(1,6), zeros(1,6));
    collIdx = int32(zeros(1,7)); collType = int32(zeros(1,7));
    dataList = zeros(1,63); radiusList = zeros(1,7);
    [~, collType, dataList, radiusList] = calllib('libHRCInterface',...
        'getUIInfoMationInterface', collIdx, collType, dataList, radiusList);
    bodyData.type = collType;
    bodyData.data = reshape(dataList, 9, 7)';
    bodyData.radius = radiusList;
    tcp_mm = soFK2_tcp(q_deg);
end

function renderSTLRobot(ax, meshData, JOINTS, q_rad, colors, alpha)
    hold(ax,'on');
    T_all = urdfFK(JOINTS, q_rad);
    for li = 1:7
        V = meshData{li}.V; F = meshData{li}.F;
        T = T_all{li}; R = T(1:3,1:3); t = T(1:3,4);
        V_w = (R * V' + t)';
        patch(ax,'Faces',F,'Vertices',V_w,'FaceColor',colors{li},'EdgeColor','none',...
            'FaceAlpha',alpha,'FaceLighting','gouraud','AmbientStrength',0.4,...
            'DiffuseStrength',0.7,'SpecularStrength',0.3);
    end
    tcp = T_all{8}(1:3,4)';  % 使用末端TCP (含elfin_end_joint偏移)
    plot3(ax,tcp(1),tcp(2),tcp(3),'rp','MarkerSize',10,'MarkerFaceColor','r');
    jpts = zeros(7,3);
    for i=1:7, jpts(i,:) = T_all{i}(1:3,4)'; end
    plot3(ax,jpts(:,1),jpts(:,2),jpts(:,3),'k-o','MarkerSize',4,'MarkerFaceColor',[0.3 0.3 0.3]);
    axis(ax,'equal'); grid(ax,'on'); camlight(ax,'headlight'); lighting(ax,'gouraud');
end

function renderSTLRobotOnBase(ax, meshData, JOINTS, q_rad, colors, alpha, Tbase)
    hold(ax,'on');
    T_all = urdfFK(JOINTS, q_rad);
    for li = 1:7
        V = meshData{li}.V; F = meshData{li}.F;
        T = Tbase * T_all{li}; R = T(1:3,1:3); t = T(1:3,4);
        V_w = (R * V' + t)';
        patch(ax,'Faces',F,'Vertices',V_w,'FaceColor',colors{li},'EdgeColor','none',...
            'FaceAlpha',alpha,'FaceLighting','gouraud','AmbientStrength',0.4,...
            'DiffuseStrength',0.7,'SpecularStrength',0.3);
    end
    tw = Tbase * T_all{8};  % 使用TCP末端 (含elfin_end_joint 134.5mm偏移)
    plot3(ax,tw(1,4),tw(2,4),tw(3,4),'rp','MarkerSize',10,'MarkerFaceColor','r');
end

function handles = renderSTLRobotHandles(ax, meshData, JOINTS, q_rad, colors, alpha, Tbase)
    T_all = urdfFK(JOINTS, q_rad);
    handles = gobjects(9,1);
    for li = 1:7
        V = meshData{li}.V; F = meshData{li}.F;
        T = Tbase * T_all{li}; R = T(1:3,1:3); t = T(1:3,4);
        V_w = (R * V' + t)';
        handles(li) = patch(ax,'Faces',F,'Vertices',V_w,'FaceColor',colors{li},...
            'EdgeColor','none','FaceAlpha',alpha,'FaceLighting','gouraud',...
            'AmbientStrength',0.4,'DiffuseStrength',0.7,'SpecularStrength',0.3);
    end
    tw = Tbase * T_all{8};  % 使用TCP末端 (含elfin_end_joint 134.5mm偏移)
    handles(8) = plot3(ax,tw(1,4),tw(2,4),tw(3,4),'rp','MarkerSize',10,'MarkerFaceColor','r');
    jpts = zeros(7,3);
    for i=1:7, jw = Tbase*T_all{i}; jpts(i,:) = jw(1:3,4)'; end
    handles(9) = plot3(ax,jpts(:,1),jpts(:,2),jpts(:,3),'k-o','MarkerSize',4,...
        'MarkerFaceColor',[0.3 0.3 0.3],'LineWidth',1);
end

%% 碰撞包络模型渲染 (使用SO库真实DH碰撞体, v1.0.0)
function [handles, tcp_world] = renderCapsuleRobotHandles(ax, q_deg, baseOffset, ~)
    % 渲染机器人碰撞包络并返回TCP世界坐标 (m)
    % v15.5: 使用 SO 库 getUIInfoMation 获取真实碰撞体位置 (mm, DH坐标系)
    %        + forwardKinematics2 获取TCP位置 (mm)
    %        替代旧版 fk2Skeleton 经验模型
    % 输入: q_deg=关节角(deg), baseOffset=[bx,by,bz] 基座世界坐标(m)
    % 输出: handles=图形句柄, tcp_world=TCP世界坐标 [x,y,z](m)
    
    bx = baseOffset(1); by = baseOffset(2); bz = baseOffset(3);
    
    if libisloaded('libHRCInterface')
        % --- SO库: 获取真实碰撞体位置 ---
        [bodyData, tcp_mm] = soGetCollisionBodies(q_deg);
        tcp_world = tcp_mm / 1000 + [bx, by, bz];
        
        capsuleColors = {[0.4 0.4 0.45], [0.2 0.4 0.8], [0.2 0.7 0.3], ...
                         [0.9 0.5 0.1], [0.6 0.2 0.8]};
        capsuleAlpha = 0.45;
        
        handles = gobjects(7, 1);  % 5碰撞体 + TCP标记 + 骨架线
        skelPts = [bx, by, bz];  % 基座原点
        
        for bi = 1:5
            if bodyData.type(bi) == 0, continue; end
            d = bodyData.data(bi, :);
            r_m = bodyData.radius(bi) / 1000;
            
            if bodyData.type(bi) == 2  % 胶囊
                p1_w = d(1:3)/1000 + [bx, by, bz];
                p2_w = d(4:6)/1000 + [bx, by, bz];
                handles(bi) = drawCapsule3D_h(ax, p1_w, p2_w, r_m, capsuleColors{bi}, capsuleAlpha);
                % 骨架: 取胶囊中点
                skelPts(end+1,:) = (p1_w + p2_w) / 2; %#ok<AGROW>
            elseif bodyData.type(bi) == 1  % 球
                c_w = d(1:3)/1000 + [bx, by, bz];
                [Xs,Ys,Zs] = sphere(10);
                handles(bi) = surf(ax, Xs*r_m+c_w(1), Ys*r_m+c_w(2), Zs*r_m+c_w(3),...
                    'FaceColor', capsuleColors{bi}, 'FaceAlpha', capsuleAlpha, 'EdgeColor','none');
                skelPts(end+1,:) = c_w; %#ok<AGROW>
            end
        end
        
        % TCP标记 (红色五角星)
        handles(6) = plot3(ax, tcp_world(1), tcp_world(2), tcp_world(3),...
            'rp','MarkerSize',14,'MarkerFaceColor','r','LineWidth',1.5);
        
        % 骨架线 (基座→碰撞体中心→TCP)
        skelPts(end+1,:) = tcp_world;
        handles(7) = plot3(ax, skelPts(:,1), skelPts(:,2), skelPts(:,3), 'k--o',...
            'MarkerSize',5,'MarkerFaceColor',[0.3 0.3 0.3],'LineWidth',1.2);
    else
        % --- 后备: 旧版FK2骨架模型 (SO库不可用时) ---
        capsuleR = [0.160, 0.140, 0.120, 0.100];
        capsuleColors = {[0.4 0.4 0.45], [0.2 0.4 0.8], [0.2 0.7 0.3], ...
                         [0.9 0.5 0.1], [0.6 0.2 0.8]};
        capsuleAlpha = 0.45;
        joints = fk2Skeleton(q_deg);
        joints_w = joints + [bx, by, bz];
        handles = gobjects(6,1);
        for li = 1:4
            handles(li) = drawCapsule3D_h(ax, joints_w(li,:), joints_w(li+1,:), ...
                capsuleR(li), capsuleColors{li}, capsuleAlpha);
        end
        tcp_world = joints_w(5,:);
        handles(5) = plot3(ax, tcp_world(1), tcp_world(2), tcp_world(3),...
            'rp','MarkerSize',14,'MarkerFaceColor','r','LineWidth',1.5);
        handles(6) = plot3(ax, joints_w(:,1), joints_w(:,2), joints_w(:,3), 'k--o',...
            'MarkerSize',5,'MarkerFaceColor',[0.3 0.3 0.3],'LineWidth',1.2);
    end
end

function h = drawCapsule3D_h(ax, p1, p2, r, col, alpha)
    % 绘制胶囊体并返回hggroup句柄 (删除句柄即清除所有子对象)
    h = hggroup('Parent', ax);
    v = p2-p1; L = norm(v);
    if L < 1e-6, return; end
    [X,Y,Z] = cylinder(r, 12);
    Z = Z*L;
    dd=[0;0;1]; td=v(:)/L;
    cp = cross(dd,td);
    if norm(cp) > 1e-6
        RR=axang2r_local([cp'/norm(cp), acos(max(-1,min(1,dot(dd,td))))]);
    else
        RR=eye(3); if dot(dd,td)<0, RR(3,3)=-1; RR(1,1)=-1; end
    end
    for i=1:numel(X)
        pt=RR*[X(i);Y(i);Z(i)]; X(i)=pt(1)+p1(1); Y(i)=pt(2)+p1(2); Z(i)=pt(3)+p1(3);
    end
    surf(X, Y, Z, 'Parent',h, 'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none',...
        'FaceLighting','gouraud','AmbientStrength',0.4);
    [Xs,Ys,Zs]=sphere(8);
    surf(Xs*r+p1(1),Ys*r+p1(2),Zs*r+p1(3),'Parent',h,'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
    surf(Xs*r+p2(1),Ys*r+p2(2),Zs*r+p2(3),'Parent',h,'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
end

function drawCapsule3D(ax, p1, p2, r, col, alpha)
    v = p2-p1; L = norm(v);
    if L < 1e-6, return; end
    [X,Y,Z] = cylinder(r, 12);
    Z = Z*L;
    dd=[0;0;1]; td=v(:)/L;
    cp = cross(dd,td);
    if norm(cp) > 1e-6
        RR=axang2r_local([cp'/norm(cp), acos(max(-1,min(1,dot(dd,td))))]);
    else
        RR=eye(3); if dot(dd,td)<0, RR(3,3)=-1; RR(1,1)=-1; end
    end
    for i=1:numel(X)
        pt=RR*[X(i);Y(i);Z(i)]; X(i)=pt(1)+p1(1); Y(i)=pt(2)+p1(2); Z(i)=pt(3)+p1(3);
    end
    surf(ax,X,Y,Z,'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
    [Xs,Ys,Zs]=sphere(8);
    surf(ax,Xs*r+p1(1),Ys*r+p1(2),Zs*r+p1(3),'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
    surf(ax,Xs*r+p2(1),Ys*r+p2(2),Zs*r+p2(3),'FaceColor',col,'FaceAlpha',alpha,'EdgeColor','none');
end

function drawInfoPanel_v15(ax, taskIdx, nTotal, q_deg, vel, tcp, dist, time_s, cjkFont, dColor, soActive, carrying, nPlaced, nBoxTotal, envColl)
    rectangle(ax,'Position',[0 0 1 1],'FaceColor',[0.97 0.97 0.99],...
        'EdgeColor',[0.5 0.5 0.7],'LineWidth',2,'Curvature',0.02);
    yP = 0.96;
    text(ax,0.5,yP,'v15 ENV COLLISION','FontSize',14,'FontWeight','bold',...
        'HorizontalAlignment','center','Color',[0.1 0.1 0.3],'FontName',cjkFont);
    yP=yP-0.035;
    if soActive, srcStr = 'libHRCInterface.so (实时)'; else, srcStr = 'C++ (预计算)'; end
    text(ax,0.5,yP,srcStr,'FontSize',9,'FontWeight','bold','HorizontalAlignment','center',...
        'Color',[0.3 0.3 0.5],'FontName',cjkFont);
    
    yP=yP-0.05;
    progW = max(0.01, 0.86*(taskIdx/nTotal));
    rectangle(ax,'Position',[0.05 yP-0.03 0.90 0.04],'FaceColor',[0.15 0.35 0.65],'EdgeColor','none','Curvature',0.3);
    rectangle(ax,'Position',[0.07 yP-0.025 progW 0.03],'FaceColor',[0.3 0.75 0.45],'EdgeColor','none','Curvature',0.3);
    text(ax,0.5,yP-0.01,sprintf('Task %d / %d  [FIFO]',taskIdx,nTotal),...
        'FontSize',12,'FontWeight','bold','HorizontalAlignment','center','Color','w','FontName',cjkFont);
    
    % Self-distance
    yP=yP-0.06;
    rectangle(ax,'Position',[0.05 yP-0.035 0.90 0.05],'FaceColor',[1 1 1],...
        'EdgeColor',dColor,'LineWidth',3,'Curvature',0.2);
    text(ax,0.5,yP-0.01,sprintf('Self: %.1f mm', dist),...
        'FontSize',14,'FontWeight','bold','HorizontalAlignment','center','Color',dColor,'FontName',cjkFont);
    
    % Env collision
    yP=yP-0.06;
    if envColl==0, ec=[0 0.6 0.2]; ecStr='CLEAR'; else, ec=[0.9 0.1 0.1]; ecStr=sprintf('%d',envColl); end
    text(ax,0.08,yP,sprintf('Env: %s', ecStr),'FontSize',12,'FontWeight','bold','Color',ec,'FontName',cjkFont);
    
    % Carrying / Placed status
    yP=yP-0.035;
    if carrying
        text(ax,0.08,yP,sprintf('搬运中 | 已放置: %d/%d', nPlaced, nBoxTotal),...
            'FontSize',10,'FontWeight','bold','Color',[0.8 0.5 0],'FontName',cjkFont);
    else
        text(ax,0.08,yP,sprintf('空夹 | 已放置: %d/%d', nPlaced, nBoxTotal),...
            'FontSize',10,'FontWeight','bold','Color',[0.4 0.4 0.4],'FontName',cjkFont);
    end
    
    % TCP
    yP=yP-0.04;
    text(ax,0.05,yP,'TCP (mm)','FontSize',11,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.025;
    labels={'X','Y','Z'}; colors_xyz={[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8]};
    for ci=1:3
        text(ax,0.08,yP,sprintf('%s: %+8.1f',labels{ci},tcp(ci)*1000),...
            'FontSize',11,'FontWeight','bold','Color',colors_xyz{ci},'FontName',cjkFont);
        yP=yP-0.022;
    end
    
    % Joint bars
    yP=yP-0.01;
    text(ax,0.05,yP,'Joint (deg)','FontSize',11,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.022;
    for ji=1:6
        barFrac=abs(q_deg(ji))/360; barW=max(0.01,barFrac*0.45);
        if q_deg(ji)>=0, bC=[0.3 0.6 0.9]; else, bC=[0.9 0.5 0.2]; end
        rectangle(ax,'Position',[0.22 yP-0.006 0.45 0.012],'FaceColor',[0.93 0.93 0.93],'EdgeColor','none');
        rectangle(ax,'Position',[0.22 yP-0.006 barW 0.012],'FaceColor',bC,'EdgeColor','none','Curvature',0.3);
        text(ax,0.06,yP,sprintf('J%d',ji),'FontSize',8,'FontWeight','bold','Color',[0.3 0.3 0.3],'FontName',cjkFont);
        text(ax,0.72,yP,sprintf('%+6.1f',q_deg(ji)),'FontSize',9,'FontWeight','bold','Color',bC,'FontName',cjkFont);
        yP=yP-0.019;
    end
    
    yP=yP-0.01;
    text(ax,0.08,yP,sprintf('Time: %.3f s', time_s),...
        'FontSize',10,'FontWeight','bold','Color',[0.2 0.2 0.2],'FontName',cjkFont);
end

function data = loadNumericData(filepath)
    fid = fopen(filepath, 'r');
    if fid==-1, error('Cannot open: %s', filepath); end
    lines = {};
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line) && ~isempty(strtrim(line)) && line(1)~='#'
            lines{end+1} = line; %#ok<AGROW>
        end
    end
    fclose(fid);
    nLines = length(lines);
    if nLines==0, data=[]; return; end
    vals = sscanf(lines{1}, '%f');
    nCols = length(vals);
    data = zeros(nLines, nCols);
    data(1,:) = vals';
    for i = 2:nLines
        vals = sscanf(lines{i}, '%f');
        if length(vals)==nCols, data(i,:)=vals'; end
    end
end

function summary = readSummaryFile(filepath)
    summary = struct();
    if ~exist(filepath,'file'), return; end
    fid = fopen(filepath, 'r');
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line) || isempty(strtrim(line)) || line(1)=='#', continue; end
        tokens = regexp(line, '^\s*([a-zA-Z_0-9]+)\s*:\s*(.+)$', 'tokens');
        if ~isempty(tokens)
            key = strtrim(tokens{1}{1});
            val = strtrim(tokens{1}{2});
            summary.(key) = val;
        end
    end
    fclose(fid);
end

function val = getField(s, fieldName, default)
    if isfield(s, fieldName), val = s.(fieldName); else, val = default; end
end

function T = makeTrans(xyz), T = eye(4); T(1:3,4) = xyz(:); end
function R = makeRotX(a), c=cos(a); s=sin(a); R=[1 0 0 0;0 c -s 0;0 s c 0;0 0 0 1]; end %#ok<DEFNU>
function R = makeRotY(a), c=cos(a); s=sin(a); R=[c 0 s 0;0 1 0 0;-s 0 c 0;0 0 0 1]; end %#ok<DEFNU>
function R = makeRotZ(a), c=cos(a); s=sin(a); R=[c -s 0 0;s c 0 0;0 0 1 0;0 0 0 1]; end
function R = makeRotRPY(rpy), R = makeRotZ(rpy(3))*makeRotY(rpy(2))*makeRotX(rpy(1)); end  % URDF: Rz*Ry*Rx

function saveFig(fig, outputDir, name)
    fn = fullfile(outputDir, [name '.png']);
    print(fig, fn, '-dpng', '-r150');
    fprintf('  Saved: %s\n', fn);
end

function R=axang2r_local(ax)
    a=ax(1:3);g=ax(4);c=cos(g);s=sin(g);t=1-c;x=a(1);y=a(2);z=a(3);
    R=[t*x*x+c t*x*y-s*z t*x*z+s*y;t*x*y+s*z t*y*y+c t*y*z-s*x;t*x*z-s*y t*y*z+s*x t*z*z+c];
end

%% ═══════════════════ 场景辅助函数 ═══════════════════

function drawGround_v11(ax,x0,x1,y0,y1)
    patch(ax,'Vertices',[x0 y0 0;x1 y0 0;x1 y1 0;x0 y1 0],'Faces',[1 2 3 4],...
          'FaceColor',[.92 .92 .90],'EdgeColor','none','FaceAlpha',0.4);
end

function drawCabinet_v11(ax,c,bx,by)
    x=bx-c.widthX/2;y=by-c.depthY/2;w=c.widthX;d=c.depthY;h=c.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',c.color,'EdgeColor',[.5 .5 .5],'FaceAlpha',.95,'LineWidth',1);
end

function drawFrame_v11(ax,f,cylN)
    r=f.tubeR;wx=f.widthX;dy=f.depthY;h=f.height;cx=f.cx;cy=f.cy;
    % corners: 1=left-near, 2=right-near, 3=right-far, 4=left-far
    c=[cx-wx/2 cy-dy/2;cx+wx/2 cy-dy/2;cx+wx/2 cy+dy/2;cx-wx/2 cy+dy/2];
    % 4根立柱
    for i=1:4, drawTube_v11(ax,c(i,1),c(i,2),0,c(i,1),c(i,2),h,r,f.color,cylN); end
    rb = r*0.8;  % 横梁半径
    % 近端面 (Y-neg): 仅底边 (无顶边, 无中间栏杆)
    drawTube_v11(ax,c(1,1),c(1,2),0.05,c(2,1),c(2,2),0.05,rb,f.color,cylN);
    % 远端面 (Y-pos): 底边 + 顶边
    drawTube_v11(ax,c(3,1),c(3,2),0.05,c(4,1),c(4,2),0.05,rb,f.color,cylN);
    drawTube_v11(ax,c(3,1),c(3,2),h-0.05,c(4,1),c(4,2),h-0.05,rb,f.color,cylN);
    % 顶部: 2根侧梁 (平行于Y轴, 连接近端→远端)
    drawTube_v11(ax,c(1,1),c(1,2),h-0.05,c(4,1),c(4,2),h-0.05,rb,f.color,cylN);  % 左侧
    drawTube_v11(ax,c(2,1),c(2,2),h-0.05,c(3,1),c(3,2),h-0.05,rb,f.color,cylN);  % 右侧
end

function drawPallet_v11(ax,pal,frm,fontName)
    x=frm.cx-pal.widthX/2;y=frm.cy-pal.depthY/2;
    w=pal.widthX;d=pal.depthY;h=pal.heightZ;
    v=[x y 0;x+w y 0;x+w y+d 0;x y+d 0;x y h;x+w y h;x+w y+d h;x y+d h];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',pal.color,'EdgeColor',[.15 .30 .60],'FaceAlpha',.90,'LineWidth',1.2);
    text(ax,frm.cx,frm.cy,h+0.03,sprintf('Pallet %dcm',round(h*100)),...
         'FontSize',12,'HorizontalAlignment','center','Color',[.05 .15 .45],...
         'FontWeight','bold','FontName',fontName);
end

function drawConveyor_v15(ax,cv,cylN)
    cx=cv.cx;cy=cv.cy;ly=cv.lengthY;wx=cv.widthX;hz=cv.heightZ;
    x0=cx-wx/2;y0=cy-ly/2;
    % 侧板
    drawBox3D_v11(ax,x0-.015,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    drawBox3D_v11(ax,x0+wx,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    % 腿 (6对)
    lw=.035;nLegs=6;
    legSpacing = ly/(nLegs+1);
    for li=1:nLegs
        yLeg = y0 + li*legSpacing;
        for s=[-1 1]
            lx=cx+s*(wx/2-.06);
            drawBox3D_v11(ax,lx-lw/2,yLeg-lw/2,0,lw,lw,hz-.01,[.25 .25 .25]);
        end
    end
    % 滚筒
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        [X,Y,Z]=cylinder(cv.rollerR,cylN); Z=Z*wx*.9-wx*.9/2;
        surf(ax,Z+cx,zeros(size(X))+ry,X+hz+cv.rollerR,'FaceColor',[.55 .55 .55],'EdgeColor','none','FaceAlpha',.6);
    end
    % 皮带
    bz=hz+cv.rollerR;
    bV=[x0+.02 y0+.04 bz;x0+wx-.02 y0+.04 bz;x0+wx-.02 y0+ly-.04 bz;x0+.02 y0+ly-.04 bz;
        x0+.02 y0+.04 bz+cv.beltH;x0+wx-.02 y0+.04 bz+cv.beltH;
        x0+wx-.02 y0+ly-.04 bz+cv.beltH;x0+.02 y0+ly-.04 bz+cv.beltH];
    patch(ax,'Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[.15 .15 .15],'FaceAlpha',.92);
end

function drawBox3D_v11(ax,x,y,z,dx,dy,dz,col)
    v=[x y z;x+dx y z;x+dx y+dy z;x y+dy z;x y z+dz;x+dx y z+dz;x+dx y+dy z+dz;x y+dy z+dz];
    patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',col,'EdgeColor','none','FaceAlpha',.95);
end

function h = drawBox_v11(ax, pos, bx)
    x=pos(1)-bx.lx/2; y=pos(2)-bx.wy/2; z=pos(3)-bx.hz/2;
    v=[x y z;x+bx.lx y z;x+bx.lx y+bx.wy z;x y+bx.wy z;
       x y z+bx.hz;x+bx.lx y z+bx.hz;x+bx.lx y+bx.wy z+bx.hz;x y+bx.wy z+bx.hz];
    h = patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',bx.color,'EdgeColor',[.35 .25 .1],'FaceAlpha',.95,'LineWidth',1.2);
end

function drawTube_v11(ax,x1,y1,z1,x2,y2,z2,radius,color,cylN)
    [X,Y,Z]=cylinder(radius,cylN);
    v=[x2-x1;y2-y1;z2-z1];l=norm(v);
    if l<.001,return;end
    Z=Z*l;dd=[0;0;1];td=v/l;cp=cross(dd,td);
    if norm(cp)>1e-6
        RR=axang2r_local([cp'/norm(cp),acos(max(-1,min(1,dot(dd,td))))]);
    else
        RR=eye(3);if dot(dd,td)<0,RR(3,3)=-1;RR(1,1)=-1;end
    end
    for i=1:numel(X)
        pt=RR*[X(i);Y(i);Z(i)];X(i)=pt(1)+x1;Y(i)=pt(2)+y1;Z(i)=pt(3)+z1;
    end
    surf(ax,X,Y,Z,'FaceColor',color,'EdgeColor','none','FaceAlpha',.85);
end
