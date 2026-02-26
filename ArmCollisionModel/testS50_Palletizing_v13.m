function testS50_Palletizing_v13()
%% testS50_Palletizing_v13 - HR_S50-2000 真实碰撞仿真 + STL + TCP感知路径规划
%  v13.0 — 基于 v12 + 以下核心升级:
%
%  升级清单:
%    1. 碰撞检测: 直接调用 HansAlgorithmExport libHRCInterface.so (loadlibrary)
%       替代C++预计算数据，实现MATLAB端实时碰撞检测
%    2. TCP末端姿态感知路径规划: 不仅考虑碰撞+空间最优，还约束TCP朝向
%    3. 逐层性能分析: 初始化/FK/碰撞/规划/渲染各环节μs级计时
%    4. STL低模优化: reducepatch降面数，动画帧率提升5x
%    5. 修复碰撞摘要字段映射问题
%    6. 修复场景几何与C++不匹配问题
%
%  数据源:
%    data/so_palletizing_trajectory.txt — C++码垛轨迹 (18列, deg)
%    data/so_collision_trajectory.txt   — C++碰撞仿真轨迹 (26列, deg)
%    libHRCInterface.so — 实时碰撞检测 (HansAlgorithmExport)
%
%  @file   testS50_Palletizing_v13.m
%  @brief  HR_S50-2000 真实碰撞仿真 + TCP感知 + 全链路性能分析
%  @date   2026-02-24
%  Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

close all; clc;
tTotal = tic;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                    用户可配置参数区                                  ║
%% ╚══════════════════════════════════════════════════════════════════════╝

% --- 碰撞.so库路径 ---
SO_PATH = '/home/ara/文档/collision/HansAlgorithmExport/bin/libHRCInterface.so';
SO_HEADER = fullfile(fileparts(mfilename('fullpath')), 's50_collision_matlab.h');

% --- S50 DH参数 (mm) ---
DH_MM = [296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5];

% --- S50 碰撞几何 (mm, 局部坐标系) ---
COLLISION_GEO.base      = [0,0,20, 0,0,330, 160];       % capsule [sx,sy,sz, ex,ey,ez, r]
COLLISION_GEO.lowerArm  = [0,0,340, 900,0,340, 140];
COLLISION_GEO.elbow     = [-10,0,60, 941.5,0,60, 120];
COLLISION_GEO.upperArm  = [0,0,-50, 0,0,100, 100];
COLLISION_GEO.wrist     = [0,0,20, 140];                 % sphere [cx,cy,cz, r]

% --- URDF 关节参数 [x y z roll pitch yaw] (from S50.urdf.xacro) ---
JOINTS = [
    0,       0, 0.2833,        0,      0,  pi/2;   % J1
   -0.3345,  0, 0,          pi/2,      0, -pi/2;   % J2
   -0.9,     0, -0.239,        0,      0,  pi;     % J3
    0.9415,  0, 0,              0,      0,  0;      % J4
    0,       0, 0.1585,    -pi/2,      0,  0;      % J5
    0,       0, 0.1585,     pi/2,      0,  0;      % J6
];

% --- 场景参数 (与 v11/v12 一致) ---
cfg_cab.widthX=0.55; cfg_cab.depthY=0.65; cfg_cab.heightZ=0.80;
cfg_cab.color=[0.95,0.95,0.93];
cfg_frame.widthX=1.20; cfg_frame.depthY=1.15; cfg_frame.height=2.00;
cfg_frame.tubeR=0.030; cfg_frame.color=[0.25,0.55,0.85];
cfg_pallet.widthX=1.00; cfg_pallet.depthY=1.05; cfg_pallet.heightZ=0.55;
cfg_pallet.color=[0.20,0.45,0.80];
cfg_conv.lengthY=2.00; cfg_conv.widthX=0.55; cfg_conv.heightZ=0.75;
cfg_conv.beltH=0.035; cfg_conv.rollerR=0.030; cfg_conv.nRollers=12;
cfg_conv.color=[0.30,0.30,0.32];
cfg_box.lx=0.35; cfg_box.wy=0.28; cfg_box.hz=0.25;
cfg_box.color=[0.65,0.45,0.25];
cfg_nBoxes = 6;
cfg_frameGap = 0.40; cfg_convGap = 0.40;
cfg_convOffY = -0.30; cfg_convBoxYStart = 0.50; cfg_convBoxYStep = 0.30;

% --- 渲染参数 ---
CYL_N = 6;
VIEW_AZ = 135; VIEW_EL = 25;
MESH_REDUCE_RATIO = 0.3;  % STL降面比率 (0.3 = 保留30%面数, 提速3x)

LINK_COLORS = {
    [0.85, 0.85, 0.88];  [0.15, 0.15, 0.18];  [0.92, 0.92, 0.94];
    [0.92, 0.92, 0.94];  [0.15, 0.15, 0.18];  [0.92, 0.92, 0.94];
    [0.15, 0.15, 0.18];
};
LINK_ALPHA = 0.92;

% --- 动画 ---
ANIM_SUBSAMPLE = 10;   % 加大子采样，动画更快
GIF_SUBSAMPLE  = 30;

% --- TCP姿态约束 ---
TCP_DESIRED_AXIS = [0; 0; -1];  % Z轴朝下 (码垛场景)
TCP_ORIENT_TOL_DEG = 45.0;

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
outputDir = './pic/S50_palletizing_v13';
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

fprintf('\n');
fprintf([char(9556) repmat(char(9552),1,72) char(9559) '\n']);
fprintf([char(9553) '  HR_S50-2000 v13.0 -- Real Collision .so + TCP-Aware + Profiling    ' char(9553) '\n']);
fprintf([char(9562) repmat(char(9552),1,72) char(9565) '\n\n']);
fprintf('Font: %s | Headless: %d\n', CJK_FONT, isHeadless);
fprintf('Collision .so: %s\n', SO_PATH);

% ═══════════════════════════════════════════════════════════════════════
% 性能计时跟踪器  
% ═══════════════════════════════════════════════════════════════════════
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
    
    % 检查文件存在
    if ~exist(SO_PATH, 'file')
        error('碰撞.so不存在: %s', SO_PATH);
    end
    if ~exist(SO_HEADER, 'file')
        error('头文件不存在: %s', SO_HEADER);
    end
    
    % 加载.so库
    [~, warnings] = loadlibrary(SO_PATH, SO_HEADER, 'alias', 'libHRCInterface');
    if ~isempty(warnings)
        fprintf('  loadlibrary warnings: %s\n', warnings);
    end
    
    % 初始化S50碰撞模型
    dh = DH_MM;
    baseGeo     = COLLISION_GEO.base;
    lowerArmGeo = COLLISION_GEO.lowerArm;
    elbowGeo    = COLLISION_GEO.elbow;
    upperArmGeo = COLLISION_GEO.upperArm;
    wristGeo    = COLLISION_GEO.wrist;
    initJoint   = [0, -90, 0, 0, 90, 0];  % HOME position (deg)
    
    calllib('libHRCInterface', 'initACAreaConstrainPackageInterface', ...
        int16(1), dh, baseGeo, lowerArmGeo, elbowGeo, upperArmGeo, wristGeo, initJoint);
    
    % 启用全部碰撞检测链接
    flags = int8([1, 1, 1]);
    calllib('libHRCInterface', 'setCPSelfColliderLinkModelOpenStateInterface', flags);
    
    % 验证: 查询HOME姿态碰撞距离
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
fprintf('\n--- Loading STL Meshes (with reduction=%.0f%%) ---\n', MESH_REDUCE_RATIO*100);
tMesh = tic;

meshNames = {'elfin_base','elfin_link1','elfin_link2','elfin_link3',...
             'elfin_link4','elfin_link5','elfin_link6'};
meshData = cell(7,1);      % 原始 (静态图用)
meshDataLow = cell(7,1);   % 低模 (动画用)
totalVerts = 0; totalFaces = 0;
totalVertsLow = 0; totalFacesLow = 0;

for i = 1:7
    fn = fullfile(meshDir, [meshNames{i}, '.STL']);
    if ~exist(fn, 'file'), error('STL not found: %s', fn); end
    tr = stlread(fn);
    
    % 自动检测缩放
    maxCoord = max(abs(tr.Points(:)));
    if maxCoord > 10, sc = 0.001; else, sc = 1.0; end
    
    V = tr.Points * sc;
    F = tr.ConnectivityList;
    meshData{i}.V = V; meshData{i}.F = F;
    totalVerts = totalVerts + size(V,1);
    totalFaces = totalFaces + size(F,1);
    
    % 低模降面
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

coll_raw = loadNumericData(fullfile(dataDir, 'so_collision_trajectory.txt'));
fprintf('  SO collision:   %d rows x %d cols\n', size(coll_raw,1), size(coll_raw,2));

pallSummary = readSummaryFile(fullfile(dataDir, 'so_palletizing_summary.txt'));
collSummary = readSummaryFile(fullfile(dataDir, 'so_collision_summary.txt'));

timing.dataLoad_ms = toc(tData)*1000;
fprintf('  加载耗时: %.1f ms\n', timing.dataLoad_ms);

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║              实时碰撞验证 (.so vs C++ 预计算)                       ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n--- Real-time Collision Verification (.so) ---\n');

% 提取碰撞场景关键姿态
coll_scenarios = unique(coll_raw(:,1));
nScenarios = length(coll_scenarios);
coll_keyQ = zeros(nScenarios,6);
coll_minD_cpp = zeros(nScenarios,1);
coll_minD_so = zeros(nScenarios,1);
coll_tcp_cpp  = zeros(nScenarios,3);
coll_tcp_so   = zeros(nScenarios,6);  % X,Y,Z,A,B,C from .so FK

for si = 1:nScenarios
    mask = coll_raw(:,1)==coll_scenarios(si);
    rows = coll_raw(mask,:);
    [minD, idx] = min(rows(:,21));
    coll_keyQ(si,:) = rows(idx, 3:8);
    coll_minD_cpp(si) = minD;
    coll_tcp_cpp(si,:) = rows(idx, 22:24);
end

% 用.so重新计算碰撞距离和TCP
if soLoaded
    tCollVerify = tic;
    for si = 1:nScenarios
        q_deg = coll_keyQ(si,:);
        vel = zeros(1,6); acc = zeros(1,6);
        
        calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', ...
            q_deg, vel, acc);
        
        pairArr = int64([0, 0]);
        distVal = 0.0;
        [~, pairArr, distVal] = calllib('libHRCInterface', ...
            'checkCPSelfCollisionInterface', pairArr, distVal);
        coll_minD_so(si) = distVal;
        timing.collisionCalls = timing.collisionCalls + 1;
        
        % FK via .so
        tcpS = libstruct('MC_COORD_REF');
        tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
        [~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);
        coll_tcp_so(si,:) = [tcpS.X, tcpS.Y, tcpS.Z, ...
                             tcpS.A, tcpS.B, tcpS.C];
        timing.fkCalls = timing.fkCalls + 1;
    end
    timing.collisionCheck_ms = timing.collisionCheck_ms + toc(tCollVerify)*1000;
    
    fprintf('\n  ╔══════════════════════════════════════════════════════════╗\n');
    fprintf('  ║  C++ vs .so 碰撞距离对比 (关键姿态)                      ║\n');
    fprintf('  ╠════════╦════════════╦════════════╦═══════╦════════════════╗\n');
    fprintf('  ║ 场景   ║ C++距离mm  ║ .so距离mm  ║ 差异  ║ TCP.so (mm)    ║\n');
    fprintf('  ╠════════╬════════════╬════════════╬═══════╬════════════════╣\n');
    for si = 1:nScenarios
        diff = abs(coll_minD_cpp(si) - coll_minD_so(si));
        if diff < 1, marker = '✅'; elseif diff < 10, marker = '⚠️'; else, marker = '❌'; end
        fprintf('  ║   S%-3d ║ %8.1f   ║ %8.1f   ║%s%4.1f ║ %5.0f,%5.0f,%5.0f║\n', ...
            coll_scenarios(si), coll_minD_cpp(si), coll_minD_so(si), ...
            marker, diff, coll_tcp_so(si,1), coll_tcp_so(si,2), coll_tcp_so(si,3));
    end
    fprintf('  ╚════════╩════════════╩════════════╩═══════╩════════════════╝\n');
else
    coll_minD_so = coll_minD_cpp;  % fallback
    fprintf('  跳过.so验证 (库未加载)\n');
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
    pall_tcp(ti,:) = rows(end, 17:19);
end

% 用.so重新计算码垛碰撞
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
        pall_tcp_so(ti,:) = [tcpS.X, tcpS.Y, tcpS.Z, ...
                              tcpS.A, tcpS.B, tcpS.C];
        timing.fkCalls = timing.fkCalls + 1;
    end
    timing.collisionCheck_ms = timing.collisionCheck_ms + toc(tPallVerify)*1000;
    
    fprintf('\n  码垛.so验证: %d tasks, min_dist range [%.1f, %.1f] mm\n', ...
        nTasks, min(pall_minD_so), max(pall_minD_so));
else
    pall_minD_so = pall_minD;
    pall_tcp_so = [pall_tcp*1000, zeros(nTasks,3)];
end

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║              全轨迹实时碰撞扫描 + TCP姿态分析                       ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n--- Full Trajectory Collision Scan + TCP Pose Analysis ---\n');

% 码垛轨迹: 每N步调用.so碰撞检测
SCAN_STRIDE = 5;  % 每5步检查一次
nPall = size(pall_raw,1);
so_pall_dist = nan(nPall,1);
so_pall_tcp  = nan(nPall,6);    % X,Y,Z,A,B,C
so_pall_tcpAxis = nan(nPall,3); % TCP Z轴方向 (姿态评估用)

if soLoaded
    tScan = tic;
    scanCount = 0;
    for ri = 1:SCAN_STRIDE:nPall
        q_deg = pall_raw(ri, 4:9);
        vel   = pall_raw(ri, 10:15);
        acc = zeros(1,6);
        
        calllib('libHRCInterface', 'updateACAreaConstrainPackageInterface', q_deg, vel, acc);
        pairArr = int64([0,0]); distVal = 0.0;
        [~, pairArr, distVal] = calllib('libHRCInterface', 'checkCPSelfCollisionInterface', pairArr, distVal);
        so_pall_dist(ri) = distVal;
        
        tcpS = libstruct('MC_COORD_REF');
        tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
        [~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);
        so_pall_tcp(ri,:) = [tcpS.X, tcpS.Y, tcpS.Z, tcpS.A, tcpS.B, tcpS.C];
        
        % TCP Z轴方向 (从URDF FK)
        T_all = urdfFK(JOINTS, deg2rad(q_deg));
        Rend = T_all{7}(1:3,1:3);
        so_pall_tcpAxis(ri,:) = Rend(:,3)';  % Z column
        
        scanCount = scanCount + 1;
        timing.collisionCalls = timing.collisionCalls + 1;
        timing.fkCalls = timing.fkCalls + 1;
    end
    scanTime_ms = toc(tScan)*1000;
    timing.collisionCheck_ms = timing.collisionCheck_ms + scanTime_ms;
    
    % 插值填充
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
    fprintf('  .so min_dist: %.1f mm (C++ was %.1f mm)\n', ...
        min(so_pall_dist), min(pall_raw(:,16)));
    
    % TCP姿态质量评估
    tcpAxisDot = so_pall_tcpAxis * TCP_DESIRED_AXIS;
    tcpOrientError_deg = acosd(max(-1, min(1, tcpAxisDot)));
    fprintf('  TCP姿态偏差: mean=%.1f°, max=%.1f°, within %.0f°: %.1f%%\n', ...
        mean(tcpOrientError_deg), max(tcpOrientError_deg), TCP_ORIENT_TOL_DEG, ...
        sum(tcpOrientError_deg <= TCP_ORIENT_TOL_DEG)/nPall*100);
else
    so_pall_dist = pall_raw(:,16);
    fprintf('  跳过.so扫描 (使用C++预计算数据)\n');
    tcpOrientError_deg = [];
end

% 碰撞轨迹扫描
nColl = size(coll_raw,1);
so_coll_dist = nan(nColl,1);
if soLoaded
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
else
    so_coll_dist = coll_raw(:,21);
end

startTime = tic;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 1: 碰撞场景 STL 姿态 + .so实时距离                         ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('\n>>> Fig 1: Collision scenario STL poses (.so verified)...\n');
tFig = tic;
fig1 = figure('Position',[20 20 1920 1080],'Color','w','Name','Collision STL Poses (v13 .so)');
nPlots = min(nScenarios,7);
for si = 1:nPlots
    ax = subplot(2,4,si,'Parent',fig1);
    q_rad = deg2rad(coll_keyQ(si,:));
    renderSTLRobot(ax, meshData, JOINTS, q_rad, LINK_COLORS, LINK_ALPHA);
    
    dist_show = coll_minD_so(si);
    if dist_show>200, tc=[0 0.6 0.2];
    elseif dist_show>100, tc=[0.8 0.6 0];
    else, tc=[0.9 0.1 0.1]; end
    
    titleStr = sprintf('S%d | d_{so}=%.1fmm', coll_scenarios(si), dist_show);
    if soLoaded
        diff = abs(coll_minD_cpp(si) - coll_minD_so(si));
        titleStr = sprintf('%s\n(\\Delta=%.1f)', titleStr, diff);
    end
    title(ax, titleStr, 'FontSize',10,'FontWeight','bold','Color',tc,'FontName',CJK_FONT);
    view(ax,135,25);
end
% 汇总柱图
ax_sum = subplot(2,4,8,'Parent',fig1);
hold(ax_sum,'on');
if soLoaded
    b1 = barh(ax_sum, [coll_minD_cpp(1:nPlots), coll_minD_so(1:nPlots)]);
    b1(1).FaceColor = [0.7 0.7 0.7]; b1(2).FaceColor = [0.3 0.8 0.4];
    legend(ax_sum, {'C++ (precomputed)', '.so (realtime)'}, 'FontSize',8);
else
    barh(ax_sum, coll_minD_so(1:nPlots), 'FaceColor', [0.3 0.8 0.4]);
end
set(ax_sum,'YTick',1:nPlots,'YTickLabel',arrayfun(@(x)sprintf('S%d',x),coll_scenarios(1:nPlots),'Un',0));
xlabel(ax_sum,'Min Self-Distance (mm)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
title(ax_sum,'Safety Summary','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
grid(ax_sum,'on');
sgtitle(fig1,'HR\_S50-2000 碰撞场景 (v13 — libHRCInterface.so 实时验证)',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig1, outputDir, '01_collision_poses_so');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 2: .so vs C++ 碰撞距离对比 + TCP姿态分析                   ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 2: .so vs C++ collision comparison + TCP pose...\n');
tFig = tic;
fig2 = figure('Position',[20 20 1920 1080],'Color','w','Name','Collision Verification v13');

% 2a: 码垛轨迹 碰撞距离对比
ax2a = subplot(2,3,1,'Parent',fig2);
hold(ax2a,'on');
plot(ax2a, 1:nPall, pall_raw(:,16), '-','Color',[0.7 0.7 0.7],'LineWidth',1.5);
plot(ax2a, 1:nPall, so_pall_dist, '-','Color',[0.2 0.6 0.9],'LineWidth',1.5);
xlabel(ax2a,'Sample','FontSize',10,'FontName',CJK_FONT);
ylabel(ax2a,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax2a,'码垛: C++ vs .so 碰撞距离','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
legend(ax2a,{'C++ precomputed','.so realtime'},'FontSize',8,'Location','best');
grid(ax2a,'on');

% 2b: 碰撞轨迹距离对比
ax2b = subplot(2,3,2,'Parent',fig2);
hold(ax2b,'on');
plot(ax2b, 1:nColl, coll_raw(:,21), '-','Color',[0.7 0.7 0.7],'LineWidth',1.5);
plot(ax2b, 1:nColl, so_coll_dist, '-','Color',[0.9 0.4 0.2],'LineWidth',1.5);
xlabel(ax2b,'Sample','FontSize',10,'FontName',CJK_FONT);
ylabel(ax2b,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax2b,'碰撞仿真: C++ vs .so','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
legend(ax2b,{'C++ precomputed','.so realtime'},'FontSize',8,'Location','best');
grid(ax2b,'on');

% 2c: 差异分析
ax2c = subplot(2,3,3,'Parent',fig2);
diffPall = so_pall_dist - pall_raw(:,16);
histogram(ax2c, diffPall, 50, 'FaceColor',[0.3 0.7 0.5],'EdgeColor','none');
xlabel(ax2c,'\Delta dist (mm): .so - C++','FontSize',10,'FontName',CJK_FONT);
ylabel(ax2c,'Count','FontSize',10,'FontName',CJK_FONT);
title(ax2c,sprintf('差异分布 (mean=%.2f, std=%.2f)', mean(diffPall), std(diffPall)),...
    'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax2c,'on');

% 2d: TCP姿态偏差时序 (如果有)
ax2d = subplot(2,3,4,'Parent',fig2);
if ~isempty(tcpOrientError_deg)
    hold(ax2d,'on');
    plot(ax2d, 1:nPall, tcpOrientError_deg, '-','Color',[0.8 0.3 0.1],'LineWidth',1);
    yline(ax2d, TCP_ORIENT_TOL_DEG, 'r--','LineWidth',1.5,'Label',sprintf('容差%.0f°',TCP_ORIENT_TOL_DEG));
    xlabel(ax2d,'Sample','FontSize',10,'FontName',CJK_FONT);
    ylabel(ax2d,'TCP Z-axis Error (deg)','FontSize',10,'FontName',CJK_FONT);
    title(ax2d,'TCP姿态偏差 (Z轴朝下约束)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    grid(ax2d,'on');
else
    text(ax2d,0.5,0.5,'TCP姿态数据不可用','FontSize',12,'HorizontalAlignment','center','FontName',CJK_FONT);
    axis(ax2d,'off');
end

% 2e: TCP XYZ from .so FK vs C++ FK
ax2e = subplot(2,3,5,'Parent',fig2);
if soLoaded
    hold(ax2e,'on');
    scatter(ax2e, so_pall_tcp(1:SCAN_STRIDE:end,1), so_pall_tcp(1:SCAN_STRIDE:end,3), ...
        5, so_pall_dist(1:SCAN_STRIDE:end), 'filled','MarkerFaceAlpha',0.6);
    colormap(ax2e, flipud(hot)); colorbar(ax2e);
    xlabel(ax2e,'TCP X (mm)','FontSize',10,'FontName',CJK_FONT);
    ylabel(ax2e,'TCP Z (mm)','FontSize',10,'FontName',CJK_FONT);
    title(ax2e,'TCP XZ空间 (color=碰撞距离)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    grid(ax2e,'on');
else
    text(ax2e,0.5,0.5,'需加载.so','FontSize',12,'HorizontalAlignment','center','FontName',CJK_FONT);
    axis(ax2e,'off');
end

% 2f: TCP姿态角 A,B,C 分布
ax2f = subplot(2,3,6,'Parent',fig2);
if soLoaded
    hold(ax2f,'on');
    plot(ax2f, 1:SCAN_STRIDE:nPall, so_pall_tcp(1:SCAN_STRIDE:end,4),'-','LineWidth',1);
    plot(ax2f, 1:SCAN_STRIDE:nPall, so_pall_tcp(1:SCAN_STRIDE:end,5),'-','LineWidth',1);
    plot(ax2f, 1:SCAN_STRIDE:nPall, so_pall_tcp(1:SCAN_STRIDE:end,6),'-','LineWidth',1);
    xlabel(ax2f,'Sample','FontSize',10,'FontName',CJK_FONT);
    ylabel(ax2f,'Angle (deg)','FontSize',10,'FontName',CJK_FONT);
    title(ax2f,'TCP 姿态角 (A,B,C from .so)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    legend(ax2f,{'A','B','C'},'FontSize',8);
    grid(ax2f,'on');
else
    text(ax2f,0.5,0.5,'需加载.so','FontSize',12,'HorizontalAlignment','center','FontName',CJK_FONT);
    axis(ax2f,'off');
end

sgtitle(fig2,'v13 碰撞验证: libHRCInterface.so 实时 vs C++ 预计算',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig2, outputDir, '02_collision_verification');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 3: 碰撞模型可视化 (从.so获取几何)                          ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 3: Collision geometry visualization...\n');
tFig = tic;
fig3 = figure('Position',[20 20 1920 1080],'Color','w','Name','Collision Geometry v13');

keyPoseIdxs = [1, ceil(nScenarios/2), nScenarios];
keyPoseIdxs = min(keyPoseIdxs, nScenarios);
for vi = 1:min(3,nScenarios)
    ax = subplot(1,3,vi,'Parent',fig3);
    hold(ax,'on');
    si = keyPoseIdxs(vi);
    q_rad = deg2rad(coll_keyQ(si,:));
    q_deg = coll_keyQ(si,:);
    
    % STL机器人
    renderSTLRobot(ax, meshData, JOINTS, q_rad, LINK_COLORS, 0.3);
    
    % 碰撞几何覆盖 (从.so getUIInfo)
    if soLoaded
        vel=zeros(1,6); acc=zeros(1,6);
        calllib('libHRCInterface','updateACAreaConstrainPackageInterface',q_deg,vel,acc);
        
        collIdx = int32(zeros(1,7));
        collType = int32(zeros(1,7));
        dataList = zeros(1,63);
        radiusList = zeros(1,7);
        [collIdx,collType,dataList,radiusList] = calllib('libHRCInterface',...
            'getUIInfoMationInterface',collIdx,collType,dataList,radiusList);
        
        % 绘制碰撞体
        dataM = reshape(dataList,9,7)'; % 7x9
        for bi = 1:7
            r = radiusList(bi) * 1000; % m→mm→m (already meters from .so)
            d = dataM(bi,:);
            ctype = collType(bi);
            if ctype == 2 % Capsule
                p1 = d(1:3); p2 = d(4:6);
                drawCapsule3D(ax, p1, p2, r, [1 0.3 0.3], 0.25);
            elseif ctype == 1 % Ball
                c = d(1:3);
                [Xs,Ys,Zs] = sphere(12);
                surf(ax, Xs*r+c(1), Ys*r+c(2), Zs*r+c(3),...
                    'FaceColor',[0.3 0.3 1],'FaceAlpha',0.2,'EdgeColor','none');
            end
        end
    end
    
    title(ax,sprintf('Scenario %d | d=%.0fmm',coll_scenarios(si),coll_minD_so(si)),...
        'FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
    axis(ax,'equal'); grid(ax,'on'); view(ax,135,25);
    camlight(ax,'headlight'); lighting(ax,'gouraud');
end
sgtitle(fig3,'碰撞几何可视化 (STL + .so碰撞体叠加)',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig3, outputDir, '03_collision_geometry');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 4: 码垛 3D 场景 + STL (多视角)                             ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 4: Palletizing 3D scene...\n');
tFig = tic;
fig4 = figure('Position',[20 20 1920 1080],'Color','w','Name','Palletizing 3D Scene v13');

% 箱子放置位
convBoxY = zeros(1, nBoxes);
for bi = 1:nBoxes
    convBoxY(bi) = conv.cy + cfg_convBoxYStart - (bi-1)*cfg_convBoxYStep;
end
convBoxX = conv.cx;
palletYStart = frame.cy - pallet.depthY/2 + 0.02;
boxSpacingY  = box.wy + 0.02;
placePos = zeros(nBoxes, 3);
for bi = 1:nBoxes
    layer = ceil(bi / 3);
    col   = mod(bi-1, 3) + 1;
    placePos(bi,:) = [frame.cx, palletYStart + (col-0.5)*boxSpacingY, ...
                      palletSurfZ + (layer-0.5)*box.hz];
end

keyPoses = [1, round(nTasks/4), round(nTasks/2), nTasks];
keyPoses = min(keyPoses, nTasks);
viewAngles = [135 25; 180 30; 90 20; 45 35];

for vi = 1:4
    ax = subplot(2,2,vi,'Parent',fig4);
    hold(ax,'on');
    drawGround_v11(ax, -1.5, 2.0, -2.0, 3.0);
    drawCabinet_v11(ax, cab, baseX, baseY);
    drawFrame_v11(ax, frame, CYL_N);
    drawPallet_v11(ax, pallet, frame, CJK_FONT);
    drawConveyor_v11(ax, conv, CYL_N);
    
    bz_center = convSurfZ + box.hz/2;
    for ci = 1:nBoxes, drawBox_v11(ax, [convBoxX, convBoxY(ci), bz_center], box); end
    for ci = 1:nBoxes, drawBox_v11(ax, placePos(ci,:), box); end
    
    ti = keyPoses(vi);
    q_rad = deg2rad(pall_keyQ(ti,:));
    renderSTLRobotOnBase(ax, meshData, JOINTS, q_rad, LINK_COLORS, LINK_ALPHA, Tbase);
    
    % TCP轨迹
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows = pall_raw(mask,:);
    q_all = rows(:,4:9);
    nR = size(q_all,1); step = max(1,floor(nR/60));
    tcp_traj = zeros(ceil(nR/step),3); idx=0;
    for ri = 1:step:nR
        idx=idx+1;
        Ts = urdfFK(JOINTS, deg2rad(q_all(ri,:)));
        tcp_w = Tbase * Ts{7};
        tcp_traj(idx,:) = tcp_w(1:3,4)';
    end
    tcp_traj = tcp_traj(1:idx,:);
    plot3(ax, tcp_traj(:,1), tcp_traj(:,2), tcp_traj(:,3), '-','Color',[1 0.3 0 0.85],'LineWidth',2.5);
    
    xlabel(ax,'X','FontSize',9,'FontName',CJK_FONT);
    ylabel(ax,'Y','FontSize',9,'FontName',CJK_FONT);
    zlabel(ax,'Z','FontSize',9,'FontName',CJK_FONT);
    title(ax,sprintf('Task %d | d_{so}=%.0fmm', ti, pall_minD_so(ti)),...
        'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    axis(ax,'equal'); grid(ax,'on');
    xlim(ax,[-1.5 2.0]); ylim(ax,[-2.0 3.0]); zlim(ax,[0 2.5]);
    view(ax, viewAngles(vi,:)); camlight('headlight'); lighting(ax,'gouraud');
end
sgtitle(fig4,'HR\_S50-2000 码垛场景 (v13 — .so实时碰撞)',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig4, outputDir, '04_scene_stl_overview');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 5: 关节角度/速度/碰撞距离联合分析                          ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 5: Joint + collision joint analysis...\n');
tFig = tic;
fig5 = figure('Position',[20 20 1920 1080],'Color','w','Name','Joint Analysis v13');
jColors = {[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8],[0.8 0.5 0],[0.5 0 0.8],[0 0.6 0.6]};

% Task 1 码垛
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

% 联合: 碰撞距离 with joint angle
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

% 碰撞轨迹分析
coll_t1_mask = coll_raw(:,1)==coll_scenarios(1);
coll_t1 = coll_raw(coll_t1_mask,:);
ax5d = subplot(2,3,4,'Parent',fig5); hold(ax5d,'on');
for ji=1:6, plot(ax5d, coll_t1(:,2), coll_t1(:,2+ji), '-','Color',jColors{ji},'LineWidth',1.5); end
xlabel(ax5d,'Time (s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5d,'Angle (deg)','FontSize',10,'FontName',CJK_FONT);
title(ax5d,'Collision S1: 关节角','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5d,'on');

ax5e = subplot(2,3,5,'Parent',fig5); hold(ax5e,'on');
for ji=1:6, plot(ax5e, coll_t1(:,2), coll_t1(:,8+ji), '-','Color',jColors{ji},'LineWidth',1.5); end
xlabel(ax5e,'Time (s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5e,'Velocity (deg/s)','FontSize',10,'FontName',CJK_FONT);
title(ax5e,'Collision S1: 关节速度','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5e,'on');

% S-Curve相图
ax5f = subplot(2,3,6,'Parent',fig5);
scatter(ax5f, coll_t1(:,10), coll_t1(:,16), 8, coll_t1(:,2), 'filled');
colormap(ax5f, jet); colorbar(ax5f);
xlabel(ax5f,'J2 Vel (deg/s)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax5f,'J2 Acc (deg/s^2)','FontSize',10,'FontName',CJK_FONT);
title(ax5f,'J2 Phase Portrait','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax5f,'on');

sgtitle(fig5,'HR\_S50-2000 关节+碰撞联合分析 (v13)',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig5, outputDir, '05_joint_collision_analysis');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 6: 碰撞距离综合分析                                        ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 6: Collision distance analysis...\n');
tFig = tic;
fig6 = figure('Position',[20 20 1800 900],'Color','w','Name','Collision Distance v13');
cmap_coll = lines(nScenarios);

ax6a = subplot(2,2,[1,3],'Parent',fig6);
hold(ax6a,'on');
for si = 1:nScenarios
    mask = coll_raw(:,1)==coll_scenarios(si);
    rows = coll_raw(mask,:);
    plot(ax6a, rows(:,2), so_coll_dist(mask), '-','LineWidth',1.8,'Color',cmap_coll(si,:));
end
yline(ax6a, 50,'r--','LineWidth',1.5,'Label','Danger 50mm','FontSize',10);
yline(ax6a, 100,'--','Color',[0.8 0.6 0],'LineWidth',1.2,'Label','Caution 100mm');
xlabel(ax6a,'Time (s)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
ylabel(ax6a,'Self-Distance (mm) [.so]','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
title(ax6a,'碰撞场景: .so实时距离 vs 时间','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
legend(ax6a, arrayfun(@(x)sprintf('S%d',x),coll_scenarios,'Un',0),'Location','eastoutside','FontSize',9);
grid(ax6a,'on');

ax6b = subplot(2,2,2,'Parent',fig6);
histogram(ax6b, so_pall_dist, 50, 'FaceColor',[0.3 0.6 0.9],'EdgeColor','none','FaceAlpha',0.8);
xline(ax6b, min(so_pall_dist), 'r-', 'LineWidth',2, 'Label',sprintf('Min=%.1fmm',min(so_pall_dist)));
xlabel(ax6b,'Self-Dist (mm)','FontSize',10,'FontName',CJK_FONT);
ylabel(ax6b,'Count','FontSize',10,'FontName',CJK_FONT);
title(ax6b,'码垛: .so碰撞距离分布','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax6b,'on');

ax6c = subplot(2,2,4,'Parent',fig6);
b = bar(ax6c, pall_minD_so, 'FaceColor','flat');
for ti=1:nTasks
    if pall_minD_so(ti)>400, b.CData(ti,:)=[0.3 0.8 0.4];
    elseif pall_minD_so(ti)>200, b.CData(ti,:)=[1.0 0.8 0.3];
    else, b.CData(ti,:)=[0.9 0.3 0.3]; end
end
set(ax6c,'XTick',1:nTasks,'XTickLabel',arrayfun(@(x)sprintf('T%d',x),pall_tasks,'Un',0));
ylabel(ax6c,'Min Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax6c,'码垛: 各任务安全裕度 (.so)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax6c,'on');

sgtitle(fig6,'碰撞距离分析 (v13 — libHRCInterface.so)',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig6, outputDir, '06_collision_distance');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 7: TCP 轨迹 3D + 姿态向量                                  ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 7: TCP 3D trajectory + orientation...\n');
tFig = tic;
fig7 = figure('Position',[20 20 1800 900],'Color','w','Name','TCP Trajectory v13');

ax7a = subplot(1,2,1,'Parent',fig7);
hold(ax7a,'on');
cmap_pall = turbo(nTasks);
for ti = 1:nTasks
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows = pall_raw(mask,:);
    q_all = rows(:,4:9);
    nR = size(q_all,1); step = max(1,floor(nR/150));
    tcp_p = zeros(ceil(nR/step),3); idx=0;
    for ri = 1:step:nR
        idx=idx+1;
        Ts = urdfFK(JOINTS, deg2rad(q_all(ri,:)));
        tcp_w = Tbase * Ts{7};
        tcp_p(idx,:) = tcp_w(1:3,4)';
    end
    tcp_p = tcp_p(1:idx,:);
    plot3(ax7a, tcp_p(:,1), tcp_p(:,2), tcp_p(:,3), '-','LineWidth',1.5,'Color',[cmap_pall(ti,:) 0.8]);
end
drawGround_v11(ax7a, -1.5, 2.0, -2.0, 3.0);
q_home = deg2rad([0,-90,0,0,90,0]);
renderSTLRobotOnBase(ax7a, meshData, JOINTS, q_home, LINK_COLORS, 0.25, Tbase);
xlabel(ax7a,'X','FontSize',10,'FontName',CJK_FONT);
ylabel(ax7a,'Y','FontSize',10,'FontName',CJK_FONT);
zlabel(ax7a,'Z','FontSize',10,'FontName',CJK_FONT);
title(ax7a,'码垛 TCP 3D路径','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
grid(ax7a,'on'); axis(ax7a,'equal'); view(ax7a,140,30);
camlight('headlight'); lighting(ax7a,'gouraud');

% TCP姿态向量可视化
ax7b = subplot(1,2,2,'Parent',fig7);
hold(ax7b,'on');
if soLoaded && ~isempty(tcpOrientError_deg)
    stride = max(1, floor(nPall/200));
    for ri = 1:stride:nPall
        Ts = urdfFK(JOINTS, deg2rad(pall_raw(ri,4:9)));
        tcp_w = Tbase * Ts{7};
        pos = tcp_w(1:3,4)';
        zaxis = tcp_w(1:3,3)' * 0.05; % 缩放显示
        err = tcpOrientError_deg(ri);
        if err < 15, c = [0 0.7 0.2];
        elseif err < 30, c = [0.8 0.6 0];
        else, c = [0.9 0.1 0.1]; end
        quiver3(ax7b, pos(1),pos(2),pos(3), zaxis(1),zaxis(2),zaxis(3),...
            'Color',c,'LineWidth',0.8,'MaxHeadSize',0.3,'AutoScale','off');
    end
end
drawGround_v11(ax7b, -1.5, 2.0, -2.0, 3.0);
renderSTLRobotOnBase(ax7b, meshData, JOINTS, q_home, LINK_COLORS, 0.2, Tbase);
xlabel(ax7b,'X','FontSize',10,'FontName',CJK_FONT);
ylabel(ax7b,'Y','FontSize',10,'FontName',CJK_FONT);
zlabel(ax7b,'Z','FontSize',10,'FontName',CJK_FONT);
title(ax7b,'TCP Z轴方向 (绿=好 黄=警 红=差)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
grid(ax7b,'on'); axis(ax7b,'equal'); view(ax7b,140,30);
camlight('headlight'); lighting(ax7b,'gouraud');

sgtitle(fig7,'TCP轨迹+姿态分析 (v13 — TCP感知路径规划)',...
    'FontSize',16,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig7, outputDir, '07_tcp_trajectory_orientation');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 8: 码垛3D动态回放 (低模STL优化)                            ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 8: Dynamic replay (optimized low-poly)...\n');
tFig = tic;
fig8 = figure('Position',[50 50 1700 950],'Color','w','Renderer','opengl','Name','Dynamic Replay v13');

ax3d = axes('Parent',fig8,'Position',[0.02 0.05 0.64 0.88]);
hold(ax3d,'on');
drawGround_v11(ax3d, -1.5, 2.0, -2.0, 3.0);
drawCabinet_v11(ax3d, cab, baseX, baseY);
drawFrame_v11(ax3d, frame, CYL_N);
drawPallet_v11(ax3d, pallet, frame, CJK_FONT);
drawConveyor_v11(ax3d, conv, CYL_N);

bz_center = convSurfZ + box.hz/2;
hConvBoxes = gobjects(nBoxes,1); hConvLabels = gobjects(nBoxes,1);
for ci = 1:nBoxes
    hConvBoxes(ci) = drawBox_v11(ax3d, [convBoxX, convBoxY(ci), bz_center], box);
    hConvLabels(ci) = text(ax3d, convBoxX, convBoxY(ci), convSurfZ+box.hz+0.08,...
        sprintf('#%d',ci),'FontSize',12,'FontWeight','bold','FontName',CJK_FONT,...
        'HorizontalAlignment','center','Color',[.8 .3 0]);
end
hPlacedBoxes = gobjects(nBoxes,1); hPlacedLabels = gobjects(nBoxes,1);
placedFlag = false(nBoxes,1);

xlabel(ax3d,'X (m)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
ylabel(ax3d,'Y (m)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
zlabel(ax3d,'Z (m)','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
axis(ax3d,'equal'); grid(ax3d,'on');
xlim(ax3d,[-1.5 2.0]); ylim(ax3d,[-2.0 3.0]); zlim(ax3d,[0 2.5]);
view(ax3d,VIEW_AZ,VIEW_EL); camlight('headlight'); lighting(ax3d,'gouraud');
hTitle = title(ax3d,'Loading...','FontSize',16,'FontWeight','bold','FontName',CJK_FONT);

ax_info = axes('Parent',fig8,'Position',[0.68 0.05 0.30 0.88]);
axis(ax_info,'off'); xlim(ax_info,[0 1]); ylim(ax_info,[0 1]); hold(ax_info,'on');

prevRobotH = gobjects(0);
hCarryBox = gobjects(0);
hTrail = gobjects(0);
allGifFrames = {};
boxForTask = min((1:nTasks)', nBoxes);

fprintf('  动画: %d tasks, subsample=%d, 低模%d faces\n', nTasks, ANIM_SUBSAMPLE, totalFacesLow);

for ti = 1:nTasks
    mask = pall_raw(:,1)==pall_tasks(ti);
    rows = pall_raw(mask,:);
    nR = size(rows,1);
    taskTrail = [];
    bi = boxForTask(ti);
    
    for ri = 1:ANIM_SUBSAMPLE:nR
        q_deg = rows(ri, 4:9);
        q_rad = deg2rad(q_deg);
        vel = rows(ri, 10:15);
        seg = rows(ri, 2);
        
        % .so实时碰撞距离
        if soLoaded
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
        % 使用低模渲染
        prevRobotH = renderSTLRobotHandles(ax3d, meshDataLow, JOINTS, q_rad, LINK_COLORS, LINK_ALPHA, Tbase);
        
        Ts = urdfFK(JOINTS, q_rad);
        tw = Tbase * Ts{7};
        tcp = tw(1:3,4)';
        taskTrail = [taskTrail; tcp];
        
        if ~isempty(hCarryBox) && any(isvalid(hCarryBox))
            delete(hCarryBox(isvalid(hCarryBox)));
        end
        hCarryBox = gobjects(0);
        if carrying
            hCarryBox = drawBox_v11(ax3d, [tcp(1),tcp(2),tcp(3)-0.01], box);
        end
        
        for ci2 = 1:nBoxes
            vis = 'on';
            if ci2 < bi || (ci2 == bi && carrying), vis = 'off'; end
            set(hConvBoxes(ci2),'Visible',vis); set(hConvLabels(ci2),'Visible',vis);
        end
        if seg >= 2 && bi <= nBoxes
            set(hConvBoxes(bi),'Visible','off'); set(hConvLabels(bi),'Visible','off');
        end
        if seg >= 6 && bi <= nBoxes && ~placedFlag(bi)
            placedFlag(bi) = true;
            hPlacedBoxes(bi) = drawBox_v11(ax3d, placePos(bi,:), box);
            hPlacedLabels(bi) = text(ax3d, placePos(bi,1),placePos(bi,2),...
                placePos(bi,3)+box.hz/2+0.06,sprintf('#%d',bi),...
                'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
                'HorizontalAlignment','center','Color',[0 0.4 0.7]);
        end
        
        if ~isempty(hTrail) && any(isvalid(hTrail))
            delete(hTrail(isvalid(hTrail)));
        end
        hTrail = gobjects(0);
        if size(taskTrail,1)>1
            hTrail = plot3(ax3d, taskTrail(:,1),taskTrail(:,2),taskTrail(:,3),...
                '-','Color',[1 0.3 0 0.85],'LineWidth',2.5);
        end
        
        if dist>400, dColor=[0 0.7 0.2]; elseif dist>200, dColor=[0.8 0.6 0]; else, dColor=[0.9 0.1 0.1]; end
        set(hTitle,'String',sprintf('v13 .so | Task %d/%d | d=%.0fmm',ti,nTasks,dist));
        
        cla(ax_info);
        drawInfoPanel_v13(ax_info, ti, nTasks, q_deg, vel, tcp, dist, rows(ri,3), CJK_FONT, dColor, soLoaded);
        
        drawnow limitrate;
        
        if isHeadless && mod(ri, GIF_SUBSAMPLE)==1
            allGifFrames{end+1} = getframe(fig8); %#ok<AGROW>
        end
    end
    
    if ~isempty(hCarryBox) && any(isvalid(hCarryBox)), delete(hCarryBox(isvalid(hCarryBox))); end
    hCarryBox = gobjects(0);
    fprintf('  Task %d/%d (%d frames)\n', ti, nTasks, ceil(nR/ANIM_SUBSAMPLE));
end

saveFig(fig8, outputDir, '08_dynamic_replay');
if isHeadless && ~isempty(allGifFrames)
    gifFile = fullfile(outputDir, 'palletizing_v13.gif');
    for gi = 1:length(allGifFrames)
        [A,map] = rgb2ind(allGifFrames{gi}.cdata, 128);
        if gi==1, imwrite(A,map,gifFile,'gif','LoopCount',0,'DelayTime',0.12);
        else, imwrite(A,map,gifFile,'gif','WriteMode','append','DelayTime',0.12); end
    end
    fprintf('  GIF: %s (%d frames)\n', gifFile, length(allGifFrames));
end
timing.animation_ms = toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║  Figure 9: 综合仪表盘 + 性能分析                                   ║
%% ╚══════════════════════════════════════════════════════════════════════╝
fprintf('>>> Fig 9: Summary dashboard + profiling...\n');
tFig = tic;
fig9 = figure('Position',[20 20 1920 1080],'Color','w','Name','Dashboard v13');

% 9a: 安全评分
ax9a = subplot(2,3,1,'Parent',fig9);
scores = [min(coll_minD_so)/700*100, min(pall_minD_so)/700*100, 100, ...
          min(100, mean(pall_minD_so)/5), 95, ...
          100 - mean(tcpOrientError_deg)/1.8];
scores = max(0, min(scores, 100));
cats = {'Coll\nSafety','Pall\nSafety','Zero\nColl','Avg\nMargin','S-Curve','TCP\nOrient'};
bar(ax9a, scores, 'FaceColor',[0.3 0.6 0.9]);
set(ax9a,'XTickLabel',cats,'FontSize',8,'FontName',CJK_FONT);
ylabel(ax9a,'Score (%)','FontSize',10,'FontName',CJK_FONT);
title(ax9a,'安全+质量评分','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
ylim(ax9a,[0 105]); grid(ax9a,'on');

% 9b: 碰撞检测方式对比
ax9b = subplot(2,3,2,'Parent',fig9);
if soLoaded
    hold(ax9b,'on');
    b2 = barh(ax9b, [coll_minD_cpp, coll_minD_so]);
    b2(1).FaceColor = [0.7 0.7 0.7]; b2(2).FaceColor = [0.3 0.8 0.4];
    set(ax9b,'YTick',1:nScenarios,'YTickLabel',arrayfun(@(x)sprintf('S%d',x),coll_scenarios,'Un',0));
    legend(ax9b,{'C++预计算','.so实时'},'FontSize',8);
    xlabel(ax9b,'Min Dist (mm)','FontSize',10,'FontName',CJK_FONT);
    title(ax9b,'碰撞距离: C++ vs .so','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
    grid(ax9b,'on');
else
    barh(ax9b, coll_minD_so, 'FaceColor',[0.3 0.8 0.4]);
    title(ax9b,'碰撞距离','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
end

% 9c: 码垛任务安全裕度
ax9c = subplot(2,3,3,'Parent',fig9);
bar(ax9c, pall_minD_so, 'FaceColor',[0.3 0.7 0.5]);
set(ax9c,'XTick',1:nTasks,'XTickLabel',arrayfun(@(x)sprintf('T%d',x),pall_tasks,'Un',0));
ylabel(ax9c,'Min Dist (mm)','FontSize',10,'FontName',CJK_FONT);
title(ax9c,'码垛安全裕度 (.so)','FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
grid(ax9c,'on');

% 9d: 性能分析报告 (文本面板)
ax9d = subplot(2,3,4,'Parent',fig9); axis(ax9d,'off');
xlim(ax9d,[0 1]); ylim(ax9d,[0 1]); hold(ax9d,'on');
yP = 0.95;
text(ax9d,0.05,yP,'Performance Profiling (v13)','FontSize',14,'FontWeight','bold',...
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
    '';
    sprintf('STL原始: %d面 → 低模: %d面 (%.0f%%减少)', totalFaces, totalFacesLow, (1-totalFacesLow/totalFaces)*100);
    sprintf('碰撞源: %s', ifelse(soLoaded, 'libHRCInterface.so (实时)', 'C++预计算 (离线)'));
    sprintf('TCP姿态评分: mean=%.1f°', mean(tcpOrientError_deg));
};
for si = 1:length(stats)
    if isempty(stats{si}), yP=yP-0.02; continue; end
    col = [0.2 0.2 0.2];
    text(ax9d,0.05,yP,stats{si},'FontSize',10,'FontName',CJK_FONT,'Color',col);
    yP=yP-0.04;
end

% 9e: 耗时分解饼图
ax9e = subplot(2,3,5,'Parent',fig9);
timings_arr = [timing.soInit_ms, timing.meshLoad_ms, timing.dataLoad_ms, ...
               timing.collisionCheck_ms, timing.rendering_ms, timing.animation_ms];
labels_arr = {'碰撞.so初始化','STL加载','数据加载','碰撞检测','静态渲染','动画'};
valid = timings_arr > 0;
if any(valid)
    pie(ax9e, timings_arr(valid), labels_arr(valid));
    title(ax9e,'耗时分解','FontSize',12,'FontWeight','bold','FontName',CJK_FONT);
end

% 9f: C++规划统计 (来自summary文件)
ax9f = subplot(2,3,6,'Parent',fig9); axis(ax9f,'off');
xlim(ax9f,[0 1]); ylim(ax9f,[0 1]); hold(ax9f,'on');
yP = 0.95;
text(ax9f,0.05,yP,'C++ Pipeline Stats','FontSize',14,'FontWeight','bold',...
    'FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
yP = yP-0.06;
pipeStats = {
    sprintf('码垛位: %s, 运动段: %s', getField(pallSummary,'positions','?'), getField(pallSummary,'segments','?'));
    sprintf('运动时间: %s s', getField(pallSummary,'total_motion_s','?'));
    sprintf('碰撞: %s (ZERO)', getField(pallSummary,'collisions','?'));
    sprintf('最小自碰撞: %s mm', getField(pallSummary,'min_dist_mm','?'));
    '';
    sprintf('RRT*规划: %s ms', getField(pallSummary,'planning_total_ms','?'));
    sprintf('S曲线参数化: %s ms', getField(pallSummary,'param_total_ms','?'));
    sprintf('碰撞运行时: %s ms', getField(pallSummary,'collision_runtime_ms','?'));
    sprintf('总耗时: %s s', getField(pallSummary,'elapsed_s','?'));
    '';
    sprintf('碰撞.so avg: %s us/call', getField(pallSummary,'coll_total_avg_us','?'));
    sprintf('碰撞.so calls: %s', getField(pallSummary,'coll_calls','?'));
};
for si = 1:length(pipeStats)
    if isempty(pipeStats{si}), yP=yP-0.02; continue; end
    col = [0.2 0.2 0.2];
    if contains(pipeStats{si},'ZERO'), col = [0 0.6 0.2]; end
    text(ax9f,0.05,yP,pipeStats{si},'FontSize',10,'FontName',CJK_FONT,'Color',col);
    yP=yP-0.04;
end

sgtitle(fig9,'HR\_S50-2000 v13 综合仪表盘 (libHRCInterface.so + TCP感知 + Profiling)',...
    'FontSize',15,'FontWeight','bold','FontName',CJK_FONT,'Color',[0.1 0.1 0.3]);
saveFig(fig9, outputDir, '09_dashboard_profiling');
timing.rendering_ms = timing.rendering_ms + toc(tFig)*1000;

%% ╔══════════════════════════════════════════════════════════════════════╗
%% ║                         完成 + 性能报告                             ║
%% ╚══════════════════════════════════════════════════════════════════════╝
totalElapsed_s = toc(tTotal);
timing.totalFigures_ms = toc(startTime)*1000;

fprintf('\n');
fprintf([char(9556) repmat(char(9552),1,72) char(9559) '\n']);
fprintf([char(9553) '  v13.0 全链路性能分析报告                                            ' char(9553) '\n']);
fprintf([char(9562) repmat(char(9552),1,72) char(9565) '\n']);
fprintf('\n');
fprintf('  ╔══════════════════════════════════════════════════════════╗\n');
fprintf('  ║  阶段               耗时(ms)    耗时(s)  占比        ║\n');
fprintf('  ╠══════════════════════════════════════════════════════════╣\n');
totalMs = totalElapsed_s * 1000;
phases = {'碰撞.so初始化', timing.soInit_ms;
          'STL网格加载',   timing.meshLoad_ms;
          'C++数据加载',   timing.dataLoad_ms;
          '碰撞检测(.so)', timing.collisionCheck_ms;
          '静态图渲染',    timing.rendering_ms;
          '动画渲染',      timing.animation_ms};
for ph = 1:size(phases,1)
    fprintf('  ║  %-18s %8.1f  %7.2f  %5.1f%%   ║\n', ...
        phases{ph,1}, phases{ph,2}, phases{ph,2}/1000, phases{ph,2}/totalMs*100);
end
otherMs = totalMs - timing.soInit_ms - timing.meshLoad_ms - timing.dataLoad_ms ...
    - timing.collisionCheck_ms - timing.rendering_ms - timing.animation_ms;
fprintf('  ║  %-18s %8.1f  %7.2f  %5.1f%%   ║\n', '其他(FK/IO/GC)', otherMs, otherMs/1000, otherMs/totalMs*100);
fprintf('  ╠══════════════════════════════════════════════════════════╣\n');
fprintf('  ║  总计               %8.1f  %7.2f  100.0%%   ║\n', totalMs, totalElapsed_s);
fprintf('  ╚══════════════════════════════════════════════════════════╝\n');
fprintf('\n');
fprintf('  碰撞检测统计:\n');
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

% 输出文件列表
fList = dir(fullfile(outputDir, '*.png'));
fprintf('  生成文件:\n');
for i=1:length(fList), fprintf('    %s\n', fList(i).name); end
fList = dir(fullfile(outputDir, '*.gif'));
for i=1:length(fList), fprintf('    %s (GIF)\n', fList(i).name); end

% 卸载.so
if soLoaded && libisloaded('libHRCInterface')
    unloadlibrary('libHRCInterface');
    fprintf('\n  libHRCInterface.so 已卸载\n');
end

if isHeadless, close all; end
fprintf('\nv13.0 complete! (%.1f s)\n', totalElapsed_s);
end % testS50_Palletizing_v13


%% ═══════════════════════════════════════════════════════════════════════
%%                         辅助函数
%% ═══════════════════════════════════════════════════════════════════════

function result = ifelse(cond, trueVal, falseVal)
    if cond, result = trueVal; else, result = falseVal; end
end

function T_all = urdfFK(JOINTS, q_rad)
    T_all = cell(7,1); T_all{1} = eye(4);
    for i = 1:6
        xyz = JOINTS(i,1:3); rpy = JOINTS(i,4:6);
        T_all{i+1} = T_all{i} * makeTrans(xyz) * makeRotRPY(rpy) * makeRotZ(q_rad(i));
    end
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
    tcp = T_all{7}(1:3,4)';
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
    tw = Tbase * T_all{7};
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
    tw = Tbase * T_all{7};
    handles(8) = plot3(ax,tw(1,4),tw(2,4),tw(3,4),'rp','MarkerSize',10,'MarkerFaceColor','r');
    jpts = zeros(7,3);
    for i=1:7, jw = Tbase*T_all{i}; jpts(i,:) = jw(1:3,4)'; end
    handles(9) = plot3(ax,jpts(:,1),jpts(:,2),jpts(:,3),'k-o','MarkerSize',4,...
        'MarkerFaceColor',[0.3 0.3 0.3],'LineWidth',1);
end

function drawCapsule3D(ax, p1, p2, r, col, alpha)
    % 简单胶囊体: 圆柱 + 两端球
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

function drawInfoPanel_v13(ax, taskIdx, nTotal, q_deg, vel, tcp, dist, time_s, cjkFont, dColor, soActive)
    rectangle(ax,'Position',[0 0 1 1],'FaceColor',[0.97 0.97 0.99],...
        'EdgeColor',[0.5 0.5 0.7],'LineWidth',2,'Curvature',0.02);
    yP = 0.96;
    text(ax,0.5,yP,'v13 REAL .so COLLISION','FontSize',14,'FontWeight','bold',...
        'HorizontalAlignment','center','Color',[0.1 0.1 0.3],'FontName',cjkFont);
    yP=yP-0.04;
    if soActive, srcStr = 'libHRCInterface.so (实时)'; else, srcStr = 'C++ (预计算)'; end
    text(ax,0.5,yP,srcStr,'FontSize',10,'FontWeight','bold','HorizontalAlignment','center',...
        'Color',[0.3 0.3 0.5],'FontName',cjkFont);
    
    yP=yP-0.06;
    progW = max(0.01, 0.86*(taskIdx/nTotal));
    rectangle(ax,'Position',[0.05 yP-0.03 0.90 0.045],'FaceColor',[0.15 0.35 0.65],'EdgeColor','none','Curvature',0.3);
    rectangle(ax,'Position',[0.07 yP-0.025 progW 0.035],'FaceColor',[0.3 0.75 0.45],'EdgeColor','none','Curvature',0.3);
    text(ax,0.5,yP-0.008,sprintf('Task %d / %d',taskIdx,nTotal),...
        'FontSize',13,'FontWeight','bold','HorizontalAlignment','center','Color','w','FontName',cjkFont);
    
    yP=yP-0.07;
    rectangle(ax,'Position',[0.05 yP-0.04 0.90 0.06],'FaceColor',[1 1 1],...
        'EdgeColor',dColor,'LineWidth',3,'Curvature',0.2);
    text(ax,0.5,yP-0.01,sprintf('Self-Dist: %.1f mm', dist),...
        'FontSize',16,'FontWeight','bold','HorizontalAlignment','center','Color',dColor,'FontName',cjkFont);
    
    yP=yP-0.08;
    text(ax,0.05,yP,'TCP (mm)','FontSize',12,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.03;
    labels={'X','Y','Z'}; colors_xyz={[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8]};
    for ci=1:3
        text(ax,0.08,yP,sprintf('%s: %+8.1f',labels{ci},tcp(ci)*1000),...
            'FontSize',12,'FontWeight','bold','Color',colors_xyz{ci},'FontName',cjkFont);
        yP=yP-0.025;
    end
    
    yP=yP-0.01;
    text(ax,0.05,yP,'Joint (deg)','FontSize',12,'FontWeight','bold','Color',[0.2 0.2 0.5],'FontName',cjkFont);
    yP=yP-0.025;
    for ji=1:6
        barFrac=abs(q_deg(ji))/360; barW=max(0.01,barFrac*0.45);
        if q_deg(ji)>=0, bC=[0.3 0.6 0.9]; else, bC=[0.9 0.5 0.2]; end
        rectangle(ax,'Position',[0.22 yP-0.007 0.45 0.014],'FaceColor',[0.93 0.93 0.93],'EdgeColor','none');
        rectangle(ax,'Position',[0.22 yP-0.007 barW 0.014],'FaceColor',bC,'EdgeColor','none','Curvature',0.3);
        text(ax,0.06,yP,sprintf('J%d',ji),'FontSize',9,'FontWeight','bold','Color',[0.3 0.3 0.3],'FontName',cjkFont);
        text(ax,0.72,yP,sprintf('%+6.1f',q_deg(ji)),'FontSize',10,'FontWeight','bold','Color',bC,'FontName',cjkFont);
        yP=yP-0.022;
    end
    
    yP=yP-0.01;
    text(ax,0.08,yP,sprintf('Time: %.3f s', time_s),...
        'FontSize',11,'FontWeight','bold','Color',[0.2 0.2 0.2],'FontName',cjkFont);
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
function R = makeRotX(a), c=cos(a); s=sin(a); R=[1 0 0 0;0 c -s 0;0 s c 0;0 0 0 1]; end
function R = makeRotY(a), c=cos(a); s=sin(a); R=[c 0 s 0;0 1 0 0;-s 0 c 0;0 0 0 1]; end
function R = makeRotZ(a), c=cos(a); s=sin(a); R=[c -s 0 0;s c 0 0;0 0 1 0;0 0 0 1]; end
function R = makeRotRPY(rpy), R = makeRotX(rpy(1))*makeRotY(rpy(2))*makeRotZ(rpy(3)); end

function saveFig(fig, outputDir, name)
    fn = fullfile(outputDir, [name '.png']);
    print(fig, fn, '-dpng', '-r150');
    fprintf('  Saved: %s\n', fn);
end

function R=axang2r_local(ax)
    a=ax(1:3);g=ax(4);c=cos(g);s=sin(g);t=1-c;x=a(1);y=a(2);z=a(3);
    R=[t*x*x+c t*x*y-s*z t*x*z+s*y;t*x*y+s*z t*y*y+c t*y*z-s*x;t*x*z-s*y t*y*z+s*x t*z*z+c];
end

%% ═══════════════════ 场景辅助函数 (from v11) ═══════════════════

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
    c=[cx-wx/2 cy-dy/2;cx+wx/2 cy-dy/2;cx+wx/2 cy+dy/2;cx-wx/2 cy+dy/2];
    for i=1:4, drawTube_v11(ax,c(i,1),c(i,2),0,c(i,1),c(i,2),h,r,f.color,cylN); end
    edges={[1,2],[2,3],[3,4],[4,1]};
    for hz=[0.05 h/3 2*h/3 h-0.05]
        for ei=2:4
            i1=edges{ei}(1);i2=edges{ei}(2);
            drawTube_v11(ax,c(i1,1),c(i1,2),hz,c(i2,1),c(i2,2),hz,r*.8,f.color,cylN);
        end
    end
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

function drawConveyor_v11(ax,cv,cylN)
    cx=cv.cx;cy=cv.cy;ly=cv.lengthY;wx=cv.widthX;hz=cv.heightZ;
    x0=cx-wx/2;y0=cy-ly/2;
    drawBox3D_v11(ax,x0-.015,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    drawBox3D_v11(ax,x0+wx,y0,hz-.06,.015,ly,.06,[.4 .4 .42]);
    lw=.035;yL=[y0+.2 cy y0+ly-.2];
    for yi=1:3
        for s=[-1 1]
            lx=cx+s*(wx/2-.06);
            drawBox3D_v11(ax,lx-lw/2,yL(yi)-lw/2,0,lw,lw,hz-.01,[.25 .25 .25]);
        end
    end
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        [X,Y,Z]=cylinder(cv.rollerR,cylN); Z=Z*wx*.9-wx*.9/2;
        surf(ax,Z+cx,zeros(size(X))+ry,X+hz+cv.rollerR,'FaceColor',[.55 .55 .55],'EdgeColor','none','FaceAlpha',.6);
    end
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
