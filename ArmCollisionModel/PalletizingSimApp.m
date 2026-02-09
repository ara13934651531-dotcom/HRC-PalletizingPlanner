function PalletizingSimApp()
%% PalletizingSimApp - HR_S50-2000 交互式码垛仿真平台 v2.0
%
%  v2.0 改进:
%    1. 性能优化: trail线用set()更新而非delete/recreate, 减少drawnow开销
%    2. 自动抓放: 添加箱子时自动确定Pick点(箱子顶部), Place点按3x2码垛排列计算
%    3. 传送带顺序: 每个新箱子自动排列在传送带不同Y位置
%    4. 用户仍可手动调整Pick/Place坐标
%
%  启动:
%    >> PalletizingSimApp()
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

%% ========================================================================
%%                         共享状态变量
%% ========================================================================
q_current = [0, -pi/3, pi/4, 0, -pi/3, 0];
baseZ = 0.80;
Tb = eye(4); Tb(3,4) = baseZ;

% DH参数 (m)
DH.d1=0.2965; DH.d2=0.3362; DH.d3=0.239;
DH.d4=0.1585; DH.d5=0.1585; DH.d6=0.1345;
DH.a2=-0.900; DH.a3=-0.9415;

jLimDeg = [-360 360; -190 10; -165 165; -360 360; -360 360; -360 360];

% 碰撞模型几何 (m)
capsuleDefs = {
    'base',     [0 0 0.03],     [0 0 0.3362],   0.13, 1;
    'lowerArm', [0 0 0.28],     [0.900 0 0.28],  0.13, 3;
    'elbow',    [-0.02 0 0.08], [0.9415 0 0.08], 0.10, 4;
    'upperArm', [0 0 -0.06],   [0 0 0.12],      0.06, 5;
};
sphereDefs = {'wrist', [0 0 0.03], 0.12, 6};

% 场景对象
sceneObjs = struct('name',{},'type',{},'pos',{},'sz',{},'color',{},'hPatch',{},'hLabel',{});

% 任务状态
pickTarget  = [];
placeTarget = [];
trailXYZ    = [];
isAnimating = false;
lastTcp     = [0 0 baseZ];   % 缓存TCP
collisionCount = 0;          % 当前动画碰撞计数

% ---- 码垛计数器 (核心新增) ----
convBoxCount   = 0;    % 传送带已添加箱子数 (用于自动Y排列)
placedBoxCount = 0;    % 已码放到篮筐的箱子数 (用于自动Place计算)
PALLET_COLS    = 3;    % 每层列数
PALLET_LAYERS  = 2;    % 层数
DEF_BOX_SZ     = [0.35, 0.28, 0.25];  % 默认箱子尺寸(m)
CONV_BOX_Y_START = 0.50;   % 第一个箱子在传送带上的Y偏移(相对convCY)
CONV_BOX_Y_STEP  = 0.32;   % 箱子间Y间距

% 默认场景配置
defScene.cab    = struct('wx',0.55,'dy',0.65,'hz',0.80,'color',[0.95 0.95 0.93]);
defScene.frame  = struct('wx',1.20,'dy',1.15,'h',2.00,'tubeR',0.030,'color',[0.25 0.55 0.85]);
defScene.pallet = struct('wx',1.00,'dy',1.05,'hz',0.55,'color',[0.20 0.45 0.80]);
defScene.conv   = struct('ly',2.00,'wx',0.55,'hz',0.75,'beltH',0.035,...
                          'rollerR',0.030,'nRollers',12,'color',[0.30 0.30 0.32]);
defScene.frameGap = 0.40;  defScene.convGap = 0.40;  defScene.convOffY = -0.30;
frameCY = defScene.cab.dy/2 + defScene.frameGap + defScene.frame.dy/2;
convCX  = defScene.cab.wx/2 + defScene.convGap + defScene.conv.wx/2;
convCY  = defScene.convOffY;
convSurfZ = defScene.conv.hz + defScene.conv.rollerR + defScene.conv.beltH;

% 渲染
CYL_N = 12;
CAPSULE_N = 8;        % 降低面数提升性能
CAPSULE_ALPHA = 0.65;
CAPSULE_COLORS = {[0.75 0.75 0.80],[0.50 0.55 0.85],[0.85 0.50 0.45],...
                  [0.50 0.80 0.55],[0.80 0.75 0.50]};
CJK_FONT = selectFont();

%% ========================================================================
%%                        图形句柄
%% ========================================================================
hCapsuleHT = gobjects(0);   capsuleLocalT = {};  capsuleLinkIdx = [];  nCapsules = 0;
hSphereHT  = gobjects(0);   sphereLocalT  = {};  sphereLinkIdx  = [];  nSpheres  = 0;
hJointMarkers = gobjects(0);
hTcpMarker    = gobjects(0);
hTrailLine    = gobjects(0);
hPickMark     = gobjects(0);
hPlaceMark    = gobjects(0);
hCarriedBox   = gobjects(0);   % 抓取中跟随TCP的箱子
hCollisionPts = gobjects(0);   % 碰撞点标记

hSliders      = gobjects(6,1);
hSliderLabels = gobjects(6,1);
hTcpText      = gobjects(0);
hStatusText   = gobjects(0);
hIKErrText    = gobjects(0);
hPalletInfo   = gobjects(0);

% Task Panel edits
hPickX = gobjects(0); hPickY = gobjects(0); hPickZ = gobjects(0);
hPlaceX = gobjects(0); hPlaceY = gobjects(0); hPlaceZ = gobjects(0);
hSpeedSlider = gobjects(0); hSpeedLabel = gobjects(0);
hObjType = gobjects(0); hObjName = gobjects(0);
hSzX = gobjects(0); hSzY = gobjects(0); hSzZ = gobjects(0);
hPosX = gobjects(0); hPosY = gobjects(0); hPosZ = gobjects(0);
hObjList = gobjects(0);

%% ========================================================================
%%                        创建主窗口
%% ========================================================================
fig = figure('Name', 'HR_S50-2000 Interactive Palletizing Simulator v2.0', ...
    'NumberTitle', 'off', 'Color', [0.94 0.94 0.96], ...
    'Position', [40 40 1750 950], 'Renderer', 'opengl', ...
    'CloseRequestFcn', @onClose, 'MenuBar', 'none', 'ToolBar', 'figure');

% ===== 3D场景 =====
ax3d = axes('Parent', fig, 'Position', [0.01 0.01 0.62 0.97]);
hold(ax3d, 'on');
xlabel(ax3d,'X (m)','FontSize',13,'FontWeight','bold','FontName',CJK_FONT);
ylabel(ax3d,'Y (m)','FontSize',13,'FontWeight','bold','FontName',CJK_FONT);
zlabel(ax3d,'Z (m)','FontSize',13,'FontWeight','bold','FontName',CJK_FONT);
set(ax3d,'FontSize',11,'FontWeight','bold','FontName',CJK_FONT);
axis(ax3d,'equal'); grid(ax3d,'on');
set(ax3d, 'GridAlpha', 0.15, 'GridColor', [0.6 0.6 0.6]);
xlim(ax3d,[-1.2 1.5]); ylim(ax3d,[-1.5 2.5]); zlim(ax3d,[0 2.2]);
view(ax3d,135,25);
camlight('headlight'); lighting(ax3d,'gouraud');
title(ax3d, 'HR\_S50-2000 Interactive Simulator v2.0', ...
    'FontSize', 16, 'FontWeight', 'bold', 'FontName', CJK_FONT, 'Color', [0.1 0.1 0.3]);
rotate3d(ax3d, 'on');

% ===== 右侧面板 =====
panelX = 0.635; panelW = 0.355;

% --- 场景编辑面板 ---
scnPnl = uipanel(fig, 'Title', 'Scene Editor', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', CJK_FONT, ...
    'Position', [panelX 0.68 panelW 0.31], ...
    'BackgroundColor', [0.96 0.96 0.98]);

yy = 0.85; dyP = 0.13;
uicontrol(scnPnl,'Style','text','String','Type:','Units','normalized',...
    'Position',[0.02 yy 0.12 0.10],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hObjType = uicontrol(scnPnl,'Style','popupmenu','String',{'Box','Cylinder','Wall'},...
    'Units','normalized','Position',[0.15 yy 0.25 0.10],'FontSize',11,'FontName',CJK_FONT);
uicontrol(scnPnl,'Style','text','String','Name:','Units','normalized',...
    'Position',[0.42 yy 0.12 0.10],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hObjName = uicontrol(scnPnl,'Style','edit','String','Box1','Units','normalized',...
    'Position',[0.55 yy+0.01 0.25 0.10],'FontSize',11,'FontName',CJK_FONT);
uicontrol(scnPnl,'Style','pushbutton','String','Add','Units','normalized',...
    'Position',[0.82 yy+0.01 0.16 0.12],'FontSize',12,'FontWeight','bold',...
    'FontName',CJK_FONT,'Callback',@addObjectCB,'BackgroundColor',[0.3 0.7 0.4],...
    'ForegroundColor','w');

yy = yy - dyP;
% 尺寸 (默认码垛箱子尺寸)
uicontrol(scnPnl,'Style','text','String','Size(m):','Units','normalized',...
    'Position',[0.02 yy 0.14 0.10],'FontSize',10,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hSzX = uicontrol(scnPnl,'Style','edit','String',num2str(DEF_BOX_SZ(1),'%.2f'),'Units','normalized','Position',[0.17 yy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hSzY = uicontrol(scnPnl,'Style','edit','String',num2str(DEF_BOX_SZ(2),'%.2f'),'Units','normalized','Position',[0.30 yy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hSzZ = uicontrol(scnPnl,'Style','edit','String',num2str(DEF_BOX_SZ(3),'%.2f'),'Units','normalized','Position',[0.43 yy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);

% 位置 (自动计算初始位置)
initBoxPos = nextConveyorPos();
uicontrol(scnPnl,'Style','text','String','Pos(m):','Units','normalized',...
    'Position',[0.02 yy-dyP 0.14 0.10],'FontSize',10,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hPosX = uicontrol(scnPnl,'Style','edit','String',num2str(initBoxPos(1),'%.2f'),'Units','normalized','Position',[0.17 yy-dyP+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hPosY = uicontrol(scnPnl,'Style','edit','String',num2str(initBoxPos(2),'%.2f'),'Units','normalized','Position',[0.30 yy-dyP+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hPosZ = uicontrol(scnPnl,'Style','edit','String',num2str(initBoxPos(3),'%.3f'),'Units','normalized','Position',[0.43 yy-dyP+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);

% 对象列表
hObjList = uicontrol(scnPnl,'Style','listbox','String',{},'Units','normalized',...
    'Position',[0.56 yy-dyP+0.01 0.27 0.22],'FontSize',10,'FontName',CJK_FONT);
uicontrol(scnPnl,'Style','pushbutton','String','Del','Units','normalized',...
    'Position',[0.83 yy-0.02 0.15 0.10],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'Callback',@delObjectCB,'BackgroundColor',[0.8 0.3 0.3],'ForegroundColor','w');
uicontrol(scnPnl,'Style','pushbutton','String','Clear','Units','normalized',...
    'Position',[0.83 yy-dyP+0.01 0.15 0.10],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'Callback',@clearSceneCB,'BackgroundColor',[0.6 0.3 0.3],'ForegroundColor','w');

% --- 机器人控制面板 ---
robPnl = uipanel(fig, 'Title', 'Robot Control', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', CJK_FONT, ...
    'Position', [panelX 0.34 panelW 0.33], ...
    'BackgroundColor', [0.96 0.96 0.98]);

jNames = {'J1','J2','J3','J4','J5','J6'};
for ji = 1:6
    yp = 0.88 - (ji-1)*0.115;
    uicontrol(robPnl,'Style','text','String',jNames{ji},'Units','normalized',...
        'Position',[0.01 yp-0.02 0.06 0.08],'FontSize',11,'FontWeight','bold',...
        'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
    hSliders(ji) = uicontrol(robPnl,'Style','slider','Units','normalized',...
        'Position',[0.08 yp 0.58 0.06],...
        'Min',jLimDeg(ji,1),'Max',jLimDeg(ji,2),...
        'Value',rad2deg(q_current(ji)),...
        'Callback',@jointSliderCB);
    hSliderLabels(ji) = uicontrol(robPnl,'Style','text','Units','normalized',...
        'Position',[0.67 yp-0.02 0.15 0.08],...
        'String',sprintf('%+.1f',rad2deg(q_current(ji))),...
        'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
        'BackgroundColor',[0.96 0.96 0.98]);
    try addlistener(hSliders(ji), 'ContinuousValueChange', @jointSliderCB); catch; end
end
hTcpText = uicontrol(robPnl,'Style','text','Units','normalized',...
    'Position',[0.02 0.01 0.55 0.08],...
    'String','TCP: X=0.0 Y=0.0 Z=0.0 mm','FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98],'ForegroundColor',[0.1 0.1 0.6]);
uicontrol(robPnl,'Style','pushbutton','String','Home','Units','normalized',...
    'Position',[0.60 0.01 0.18 0.09],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@homePoseCB,'BackgroundColor',[0.3 0.5 0.8],'ForegroundColor','w');
uicontrol(robPnl,'Style','pushbutton','String','Zero','Units','normalized',...
    'Position',[0.80 0.01 0.18 0.09],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@zeroPoseCB,'BackgroundColor',[0.5 0.5 0.6],'ForegroundColor','w');

% --- 任务控制面板 ---
taskPnl = uipanel(fig, 'Title', 'Task Control', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', CJK_FONT, ...
    'Position', [panelX 0.04 panelW 0.29], ...
    'BackgroundColor', [0.96 0.96 0.98]);

% Pick目标
uicontrol(taskPnl,'Style','text','String','Pick(m):','Units','normalized',...
    'Position',[0.02 0.85 0.14 0.10],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hPickX = uicontrol(taskPnl,'Style','edit','String',num2str(convCX,'%.2f'),'Units','normalized','Position',[0.17 0.86 0.11 0.10],'FontSize',10,'FontName',CJK_FONT);
hPickY = uicontrol(taskPnl,'Style','edit','String','0.20','Units','normalized','Position',[0.29 0.86 0.11 0.10],'FontSize',10,'FontName',CJK_FONT);
hPickZ = uicontrol(taskPnl,'Style','edit','String',num2str(convSurfZ+DEF_BOX_SZ(3),'%.3f'),'Units','normalized','Position',[0.41 0.86 0.11 0.10],'FontSize',10,'FontName',CJK_FONT);
uicontrol(taskPnl,'Style','pushbutton','String','Set','Units','normalized',...
    'Position',[0.54 0.85 0.14 0.12],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@setPickCB,'BackgroundColor',[0.2 0.7 0.3],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','TCP','Units','normalized',...
    'Position',[0.69 0.85 0.14 0.12],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@usePickTcpCB,'BackgroundColor',[0.4 0.6 0.3],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','Auto','Units','normalized',...
    'Position',[0.84 0.85 0.14 0.12],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@autoPickCB,'BackgroundColor',[0.5 0.7 0.2],'ForegroundColor','w',...
    'TooltipString','Auto-detect pick from last added box');

% Place目标
uicontrol(taskPnl,'Style','text','String','Place(m):','Units','normalized',...
    'Position',[0.02 0.71 0.14 0.10],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hPlaceX = uicontrol(taskPnl,'Style','edit','String','0.00','Units','normalized','Position',[0.17 0.72 0.11 0.10],'FontSize',10,'FontName',CJK_FONT);
hPlaceY = uicontrol(taskPnl,'Style','edit','String',num2str(frameCY-defScene.pallet.dy/2+0.02+DEF_BOX_SZ(2)/2,'%.3f'),'Units','normalized','Position',[0.29 0.72 0.11 0.10],'FontSize',10,'FontName',CJK_FONT);
hPlaceZ = uicontrol(taskPnl,'Style','edit','String',num2str(defScene.pallet.hz+DEF_BOX_SZ(3)/2,'%.3f'),'Units','normalized','Position',[0.41 0.72 0.11 0.10],'FontSize',10,'FontName',CJK_FONT);
uicontrol(taskPnl,'Style','pushbutton','String','Set','Units','normalized',...
    'Position',[0.54 0.71 0.14 0.12],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@setPlaceCB,'BackgroundColor',[0.7 0.3 0.2],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','TCP','Units','normalized',...
    'Position',[0.69 0.71 0.14 0.12],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@usePlaceTcpCB,'BackgroundColor',[0.6 0.4 0.3],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','Auto','Units','normalized',...
    'Position',[0.84 0.71 0.14 0.12],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@autoPlaceCB,'BackgroundColor',[0.7 0.5 0.2],'ForegroundColor','w',...
    'TooltipString','Auto-calculate next pallet position');

% 执行按钮行
uicontrol(taskPnl,'Style','pushbutton','String','Auto Pick & Place','Units','normalized',...
    'Position',[0.02 0.50 0.40 0.16],'FontSize',12,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@autoExecuteCB,'BackgroundColor',[0.10 0.50 0.75],'ForegroundColor','w',...
    'TooltipString','Auto-detect pick/place and execute full cycle');

uicontrol(taskPnl,'Style','pushbutton','String','Execute','Units','normalized',...
    'Position',[0.44 0.50 0.22 0.16],'FontSize',12,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@executeCB,'BackgroundColor',[0.15 0.55 0.80],'ForegroundColor','w');

uicontrol(taskPnl,'Style','pushbutton','String','Stop','Units','normalized',...
    'Position',[0.68 0.50 0.14 0.16],'FontSize',12,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@stopCB,'BackgroundColor',[0.8 0.2 0.2],'ForegroundColor','w');

uicontrol(taskPnl,'Style','text','String','Speed:','Units','normalized',...
    'Position',[0.84 0.56 0.14 0.08],'FontSize',10,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hSpeedSlider = uicontrol(taskPnl,'Style','slider','Units','normalized',...
    'Position',[0.84 0.50 0.14 0.06],'Min',5,'Max',40,'Value',15);
hSpeedLabel = uicontrol(taskPnl,'Style','text','Units','normalized',...
    'Position',[0.90 0.44 0.08 0.06],'String','15','FontSize',9,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
try addlistener(hSpeedSlider,'ContinuousValueChange',@(s,~) set(hSpeedLabel,'String',num2str(round(s.Value)))); catch; end

% 底行
uicontrol(taskPnl,'Style','pushbutton','String','Clear Trail','Units','normalized',...
    'Position',[0.02 0.28 0.22 0.14],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@clearTrailCB,'BackgroundColor',[0.6 0.5 0.3],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','Move Pick','Units','normalized',...
    'Position',[0.26 0.28 0.22 0.14],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@moveToPickCB,'BackgroundColor',[0.3 0.6 0.4],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','Move Place','Units','normalized',...
    'Position',[0.50 0.28 0.22 0.14],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@moveToPlaceCB,'BackgroundColor',[0.6 0.3 0.4],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','Reset Count','Units','normalized',...
    'Position',[0.74 0.28 0.24 0.14],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@resetCountCB,'BackgroundColor',[0.5 0.5 0.6],'ForegroundColor','w',...
    'TooltipString','Reset palletizing counter to 0');

hIKErrText = uicontrol(taskPnl,'Style','text','Units','normalized',...
    'Position',[0.02 0.14 0.55 0.10],...
    'String','IK Error: -- mm','FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98],'ForegroundColor',[0.5 0.1 0.1]);

hPalletInfo = uicontrol(taskPnl,'Style','text','Units','normalized',...
    'Position',[0.02 0.02 0.96 0.10],...
    'String',sprintf('Palletized: 0/%d | Conv boxes: 0 | Pattern: %dx%d', ...
        PALLET_COLS*PALLET_LAYERS, PALLET_COLS, PALLET_LAYERS),...
    'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.92 0.95 1.0],'ForegroundColor',[0.1 0.3 0.6]);

% --- 状态栏 ---
hStatusText = uicontrol(fig,'Style','text','Units','normalized',...
    'Position',[0.01 0.001 0.98 0.038],...
    'String','Ready | HR_S50-2000 Interactive Simulator v2.0','FontSize',12,...
    'FontWeight','bold','FontName',CJK_FONT,...
    'BackgroundColor',[0.15 0.35 0.60],'ForegroundColor','w',...
    'HorizontalAlignment','center');

%% ========================================================================
%%                     初始化
%% ========================================================================
setStatus('Initializing scene...');
drawStaticScene();
initRobotModel();
updateRobotDisplay();
updatePalletInfo();
% Place位置在添加第一个箱子时自动设定, 启动时不显示标记
setStatus('Ready - Add boxes, then use Auto Pick & Place');

%% ========================================================================
%%                    自动位置计算函数
%% ========================================================================
function pos = nextConveyorPos()
    % 下一个箱子在传送带上的位置 (基于当前convBoxCount)
    bx = convCX;
    by = convCY + CONV_BOX_Y_START - convBoxCount * CONV_BOX_Y_STEP;
    % 限制在传送带范围内
    convYMin = convCY - defScene.conv.ly/2 + 0.10;
    convYMax = convCY + defScene.conv.ly/2 - 0.10;
    by = max(convYMin, min(convYMax, by));
    bz = convSurfZ + DEF_BOX_SZ(3)/2;
    pos = [bx, by, bz];
end

function pos = nextPalletPos()
    % 下一个码垛放置位置 (基于 placedBoxCount)
    palletYStart = frameCY - defScene.pallet.dy/2 + 0.02;
    boxSpacingY  = DEF_BOX_SZ(2) + 0.02;  % box width + gap
    maxBoxes = PALLET_COLS * PALLET_LAYERS;
    idx = min(placedBoxCount, maxBoxes - 1);  % 钗位防止超出层数
    layer = floor(idx / PALLET_COLS) + 1;
    col   = mod(idx, PALLET_COLS) + 1;
    px = 0;  % frame center X
    py = palletYStart + (col - 0.5) * boxSpacingY;
    pz = defScene.pallet.hz + layer * DEF_BOX_SZ(3);  % 箱子顶部 (TCP接触点)
    pos = [px, py, pz];
    if placedBoxCount >= maxBoxes
        setStatus(sprintf('WARNING: Pallet full (%d/%d)! Reset Count to continue', placedBoxCount, maxBoxes));
    end
end

function autoFillPick()
    % 自动设定Pick点: 最新添加的箱子顶部
    if isempty(sceneObjs)
        setStatus('No boxes on conveyor - add a box first');
        return;
    end
    % 找最后一个Box类型的对象
    lastBoxIdx = [];
    for ii = length(sceneObjs):-1:1
        if strcmp(sceneObjs(ii).type, 'Box')
            lastBoxIdx = ii; break;
        end
    end
    if isempty(lastBoxIdx)
        setStatus('No Box in scene'); return;
    end
    obj = sceneObjs(lastBoxIdx);
    pickPos = [obj.pos(1), obj.pos(2), obj.pos(3) + obj.sz(3)/2];
    hPickX.String = sprintf('%.3f', pickPos(1));
    hPickY.String = sprintf('%.3f', pickPos(2));
    hPickZ.String = sprintf('%.3f', pickPos(3));
    setPickCB([],[]);
end

function autoFillPlace()
    % 自动设定Place点: 下一个码垛位置
    pos = nextPalletPos();
    hPlaceX.String = sprintf('%.3f', pos(1));
    hPlaceY.String = sprintf('%.3f', pos(2));
    hPlaceZ.String = sprintf('%.3f', pos(3));
    setPlaceCB([],[]);
end

function updatePalletInfo()
    total = PALLET_COLS * PALLET_LAYERS;
    set(hPalletInfo, 'String', sprintf('Palletized: %d/%d | Conv boxes: %d | Pattern: %dx%d', ...
        placedBoxCount, total, convBoxCount, PALLET_COLS, PALLET_LAYERS));
end

function updateNextBoxPos()
    % 更新默认添加位置为下一个传送带位置
    pos = nextConveyorPos();
    hPosX.String = num2str(pos(1),'%.2f');
    hPosY.String = num2str(pos(2),'%.2f');
    hPosZ.String = num2str(pos(3),'%.3f');
end

%% ========================================================================
%%                       FK / IK
%% ========================================================================
function T_all = FK_local(qin)
    dhTab = [qin(1), DH.d1,    0,      pi/2;
             qin(2), 0,        DH.a2,  0;
             qin(3), 0,        DH.a3,  0;
             qin(4), DH.d2-DH.d3+DH.d4, 0, pi/2;
             qin(5), DH.d5,    0,      -pi/2;
             qin(6), DH.d6,    0,      0];
    Tlinks = cell(6,1);
    for ii=1:6
        ct=cos(dhTab(ii,1)); st=sin(dhTab(ii,1));
        ca=cos(dhTab(ii,4)); sa=sin(dhTab(ii,4));
        Tlinks{ii}=[ct -st*ca  st*sa  dhTab(ii,3)*ct;
                     st  ct*ca -ct*sa  dhTab(ii,3)*st;
                     0   sa     ca     dhTab(ii,2);
                     0   0      0      1];
    end
    T00=eye(4); T01=Tlinks{1}; T02=T01*Tlinks{2}; T03=T02*Tlinks{3};
    T04=T03*Tlinks{4}; T05=T04*Tlinks{5}; T06=T05*Tlinks{6};
    T_all = {T00, T01, T02, T03, T04, T05, T06};
end

function [qSol, ikErr] = solveIK(targetXYZ)
    tx = targetXYZ(1); ty = targetXYZ(2); tz = targetXYZ(3);
    q1 = atan2(-ty, -tx);
    tr = sqrt(tx^2+ty^2); tzr = tz - baseZ;
    eb=inf; q2b=-pi/2; q3b=pi/2; q4b=-pi/2;
    for q2d=-175:3:-30
        for q3d=10:3:165
            for q4d=[-150,-135,-120,-90,-60,-45]
                q2=deg2rad(q2d); q3=deg2rad(q3d); q4=deg2rad(q4d);
                T_all=FK_local([q1,q2,q3,q4,-pi/2,0]);
                T=T_all{7};
                r_fk=sqrt(T(1,4)^2+T(2,4)^2); z_fk=T(3,4);
                pe=sqrt((r_fk-tr)^2+(z_fk-tzr)^2);
                ez=T(1:3,3); zd=acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                tot=pe+0.08*zd;
                if tot<eb && zd<30*pi/180, eb=pe; q2b=q2; q3b=q3; q4b=q4; end
            end
        end
    end
    q2c=rad2deg(q2b); q3c=rad2deg(q3b); q4c=rad2deg(q4b);
    for q2d=(q2c-5):0.5:(q2c+5)
        for q3d=(q3c-5):0.5:(q3c+5)
            for q4d=(q4c-10):2:(q4c+10)
                if q2d<-175||q2d>-30||q3d<10||q3d>165, continue; end
                q2=deg2rad(q2d); q3=deg2rad(q3d); q4=deg2rad(q4d);
                T_all=FK_local([q1,q2,q3,q4,-pi/2,0]);
                T=T_all{7};
                r_fk=sqrt(T(1,4)^2+T(2,4)^2); z_fk=T(3,4);
                pe=sqrt((r_fk-tr)^2+(z_fk-tzr)^2);
                ez=T(1:3,3); zd=acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                tot=pe+0.08*zd;
                if tot<eb && zd<30*pi/180, eb=pe; q2b=q2; q3b=q3; q4b=q4; end
            end
        end
    end
    qSol = [q1, q2b, q3b, q4b, -pi/2, 0];
    ikErr = eb;
end

%% ========================================================================
%%                     机器人模型 (hgtransform)
%% ========================================================================
function initRobotModel()
    nCapsules = size(capsuleDefs, 1);
    hCapsuleHT = gobjects(nCapsules, 1);
    capsuleLocalT = cell(nCapsules, 1);
    capsuleLinkIdx = zeros(nCapsules, 1);
    for ci = 1:nCapsules
        p1 = capsuleDefs{ci,2}; p2 = capsuleDefs{ci,3};
        radius = capsuleDefs{ci,4}; linkId = capsuleDefs{ci,5};
        capsuleLinkIdx(ci) = linkId;
        ht = hgtransform('Parent', ax3d);
        hCapsuleHT(ci) = ht;
        L = norm(p2 - p1);
        [Xc,Yc,Zc] = capsuleMesh(radius, L, CAPSULE_N);
        col = CAPSULE_COLORS{min(ci, length(CAPSULE_COLORS))};
        surf(Xc, Yc, Zc, 'Parent', ht, 'FaceColor', col, 'EdgeColor', 'none', ...
            'FaceAlpha', CAPSULE_ALPHA, 'FaceLighting', 'gouraud');
        mid = (p1 + p2) / 2;
        if L > 1e-6, R = rotZtoDirection((p2-p1)/L); else, R = eye(3); end
        Tlocal = eye(4); Tlocal(1:3,1:3) = R; Tlocal(1:3,4) = mid(:);
        capsuleLocalT{ci} = Tlocal;
    end
    nSpheres = size(sphereDefs, 1);
    hSphereHT = gobjects(nSpheres, 1);
    sphereLocalT = cell(nSpheres, 1);
    sphereLinkIdx = zeros(nSpheres, 1);
    for si = 1:nSpheres
        offset = sphereDefs{si,2}; radius = sphereDefs{si,3}; linkId = sphereDefs{si,4};
        sphereLinkIdx(si) = linkId;
        ht = hgtransform('Parent', ax3d); hSphereHT(si) = ht;
        [Xs,Ys,Zs] = sphere(CAPSULE_N);
        Xs=Xs*radius; Ys=Ys*radius; Zs=Zs*radius;
        surf(Xs, Ys, Zs, 'Parent', ht, 'FaceColor', [0.80 0.75 0.50], ...
            'EdgeColor', 'none', 'FaceAlpha', CAPSULE_ALPHA, 'FaceLighting', 'gouraud');
        Tlocal = eye(4); Tlocal(1:3,4) = offset(:); sphereLocalT{si} = Tlocal;
    end
    hTcpMarker = plot3(ax3d, 0, 0, baseZ, 'rp', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    hJointMarkers = scatter3(ax3d, zeros(8,1), zeros(8,1), zeros(8,1), ...
        50, 'k', 'filled', 'MarkerEdgeColor', [0.3 0.3 0.3]);
end

%% ========================================================================
%%         更新机器人 (只改 Matrix, 极快) — 返回TCP用于缓存
%% ========================================================================
function tcp = updateRobotDisplay()
    T_all = FK_local(q_current);
    Tw = cell(7,1);
    for ii = 1:7, Tw{ii} = Tb * T_all{ii}; end
    for ci = 1:nCapsules
        set(hCapsuleHT(ci), 'Matrix', Tw{capsuleLinkIdx(ci)} * capsuleLocalT{ci});
    end
    for si = 1:nSpheres
        set(hSphereHT(si), 'Matrix', Tw{sphereLinkIdx(si)} * sphereLocalT{si});
    end
    tcp = Tw{7}(1:3,4)';
    lastTcp = tcp;
    set(hTcpMarker, 'XData', tcp(1), 'YData', tcp(2), 'ZData', tcp(3));
    jp = zeros(8,3); jp(1,:) = [0, 0, baseZ];
    for ii = 1:7, jp(ii+1,:) = Tw{ii}(1:3,4)'; end
    set(hJointMarkers, 'XData', jp(:,1), 'YData', jp(:,2), 'ZData', jp(:,3));
    set(hTcpText, 'String', sprintf('TCP: X=%+.0f Y=%+.0f Z=%+.0f mm', ...
        tcp(1)*1000, tcp(2)*1000, tcp(3)*1000));
    % 更新跟随TCP的搬运箱
    if ~isempty(hCarriedBox) && all(isvalid(hCarriedBox))
        Tcb = eye(4); Tcb(1:3,4) = tcp(:);
        set(hCarriedBox, 'Matrix', Tcb);
    end
    % 注意: 不在这里drawnow, 由调用者控制
end

%% ========================================================================
%%                         回调函数
%% ========================================================================
function jointSliderCB(~,~)
    for ji = 1:6
        q_current(ji) = deg2rad(hSliders(ji).Value);
        set(hSliderLabels(ji), 'String', sprintf('%+.1f', hSliders(ji).Value));
    end
    updateRobotDisplay();
    drawnow limitrate;
end

function homePoseCB(~,~), setJoints([0,-60,45,0,-60,0]); end
function zeroPoseCB(~,~), setJoints([0,0,0,0,0,0]); end

function setJoints(q_deg)
    for ji = 1:6
        q_current(ji) = deg2rad(q_deg(ji));
        set(hSliders(ji), 'Value', q_deg(ji));
        set(hSliderLabels(ji), 'String', sprintf('%+.1f', q_deg(ji)));
    end
    updateRobotDisplay();
    drawnow;
end

function addObjectCB(~,~)
    types = {'Box','Cylinder','Wall'};
    tp = types{hObjType.Value};
    nm = hObjName.String;
    sx = str2double(hSzX.String); sy = str2double(hSzY.String); sz = str2double(hSzZ.String);
    px = str2double(hPosX.String); py = str2double(hPosY.String); pz = str2double(hPosZ.String);
    if any(isnan([sx sy sz px py pz]))
        setStatus('Error: Invalid size or position values'); return;
    end
    colors = {[0.65 0.45 0.25],[0.72 0.55 0.30],[0.58 0.42 0.22],...
              [0.68 0.50 0.28],[0.62 0.48 0.26]};
    cidx = mod(length(sceneObjs), length(colors)) + 1;

    obj.name = nm; obj.type = tp; obj.pos = [px py pz];
    obj.sz = [sx sy sz]; obj.color = colors{cidx};
    switch tp
        case 'Box'
            obj.hPatch = drawBox3D(ax3d, px-sx/2, py-sy/2, pz-sz/2, sx, sy, sz, obj.color);
        case 'Cylinder'
            [Xc2,Yc2,Zc2] = cylinder(sx/2, 12);
            Zc2 = Zc2 * sz;
            obj.hPatch = surf(ax3d, Xc2+px, Yc2+py, Zc2+pz-sz/2, 'FaceColor', obj.color, ...
                'EdgeColor', 'none', 'FaceAlpha', 0.9);
        case 'Wall'
            obj.hPatch = drawBox3D(ax3d, px-sx/2, py-sy/2, 0, sx, sy, sz, [0.7 0.7 0.7]);
    end
    obj.hLabel = text(ax3d, px, py, pz+sz/2+0.06, nm, 'FontSize', 10, 'FontWeight', 'bold', ...
        'FontName', CJK_FONT, 'HorizontalAlignment', 'center', 'Color', [0.8 0.3 0]);
    sceneObjs(end+1) = obj;
    refreshObjList();

    % ---- 自动逻辑: 如果是Box, 递增传送带计数, 更新下一个位置 ----
    if strcmp(tp, 'Box')
        convBoxCount = convBoxCount + 1;
        updateNextBoxPos();
        % 自动设定Pick为此箱子顶部
        pickPos = [px, py, pz + sz/2];
        hPickX.String = sprintf('%.3f', pickPos(1));
        hPickY.String = sprintf('%.3f', pickPos(2));
        hPickZ.String = sprintf('%.3f', pickPos(3));
        setPickCB([],[]);
        % 自动设定Place
        autoFillPlace();
    end

    hObjName.String = sprintf('%s%d', tp, length(sceneObjs)+1);
    updatePalletInfo();
    setStatus(sprintf('Added %s "%s" at [%.2f,%.2f,%.2f] | Pick/Place auto-set', tp, nm, px, py, pz));
end

function delObjectCB(~,~)
    idx = hObjList.Value;
    if isempty(sceneObjs) || idx < 1 || idx > length(sceneObjs), return; end
    obj = sceneObjs(idx);
    nm = obj.name;
    % 如果删除的是Box, 递减传送带计数
    if strcmp(obj.type, 'Box') && convBoxCount > 0
        convBoxCount = convBoxCount - 1;
    end
    % 如果Pick标记指向已删除对象, 清除
    if ~isempty(pickTarget)
        if norm(obj.pos(1:2) - pickTarget(1:2)) < 0.05
            pickTarget = [];
            deleteValid(hPickMark); hPickMark = gobjects(0);
        end
    end
    if isvalid(obj.hPatch), delete(obj.hPatch); end
    if isvalid(obj.hLabel), delete(obj.hLabel); end
    sceneObjs(idx) = [];
    if idx > length(sceneObjs), hObjList.Value = max(1, length(sceneObjs)); end
    refreshObjList();
    updateNextBoxPos();
    updatePalletInfo();
    setStatus(sprintf('Deleted "%s"', nm));
end

function clearSceneCB(~,~)
    for ii = 1:length(sceneObjs)
        if isvalid(sceneObjs(ii).hPatch), delete(sceneObjs(ii).hPatch); end
        if isvalid(sceneObjs(ii).hLabel), delete(sceneObjs(ii).hLabel); end
    end
    sceneObjs = struct('name',{},'type',{},'pos',{},'sz',{},'color',{},'hPatch',{},'hLabel',{});
    convBoxCount = 0;
    placedBoxCount = 0;
    pickTarget = []; placeTarget = [];
    deleteValid(hPickMark); hPickMark = gobjects(0);
    deleteValid(hPlaceMark); hPlaceMark = gobjects(0);
    deleteValid(hCarriedBox); hCarriedBox = gobjects(0);
    refreshObjList();
    updateNextBoxPos();
    updatePalletInfo();
    setStatus('Scene cleared, all counters reset');
end

function refreshObjList()
    names = {};
    for ii = 1:length(sceneObjs)
        names{end+1} = sprintf('%s [%s] (%.2f,%.2f)', sceneObjs(ii).name, ...
            sceneObjs(ii).type, sceneObjs(ii).pos(1), sceneObjs(ii).pos(2)); %#ok<AGROW>
    end
    hObjList.String = names;
end

function setPickCB(~,~)
    pickTarget = [str2double(hPickX.String), str2double(hPickY.String), str2double(hPickZ.String)];
    if any(isnan(pickTarget)), setStatus('Error: Invalid pick coordinates'); pickTarget=[]; return; end
    deleteValid(hPickMark);
    hPickMark = plot3(ax3d, pickTarget(1), pickTarget(2), pickTarget(3), ...
        'gv', 'MarkerSize', 16, 'LineWidth', 3, 'MarkerFaceColor', [0 0.8 0]);
    drawnow limitrate;
end

function setPlaceCB(~,~)
    placeTarget = [str2double(hPlaceX.String), str2double(hPlaceY.String), str2double(hPlaceZ.String)];
    if any(isnan(placeTarget)), setStatus('Error: Invalid place coordinates'); placeTarget=[]; return; end
    deleteValid(hPlaceMark);
    hPlaceMark = plot3(ax3d, placeTarget(1), placeTarget(2), placeTarget(3), ...
        'rv', 'MarkerSize', 16, 'LineWidth', 3, 'MarkerFaceColor', [0.8 0 0]);
    drawnow limitrate;
end

function usePickTcpCB(~,~)
    hPickX.String = sprintf('%.3f', lastTcp(1));
    hPickY.String = sprintf('%.3f', lastTcp(2));
    hPickZ.String = sprintf('%.3f', lastTcp(3));
    setPickCB([],[]);
end

function usePlaceTcpCB(~,~)
    hPlaceX.String = sprintf('%.3f', lastTcp(1));
    hPlaceY.String = sprintf('%.3f', lastTcp(2));
    hPlaceZ.String = sprintf('%.3f', lastTcp(3));
    setPlaceCB([],[]);
end

function autoPickCB(~,~)
    autoFillPick();
end

function autoPlaceCB(~,~)
    autoFillPlace();
end

function resetCountCB(~,~)
    placedBoxCount = 0;
    convBoxCount = 0;
    updatePalletInfo();
    updateNextBoxPos();
    autoFillPlace();
    setStatus('Counters reset to 0');
end

function moveToPickCB(~,~)
    if isempty(pickTarget), setStatus('Set pick target first'); return; end
    setStatus('Solving IK for pick...');
    drawnow;
    [qSol, ikErr] = solveIK(pickTarget);
    set(hIKErrText, 'String', sprintf('IK: %.1f mm', ikErr*1000));
    animateToJoints(qSol);
    setStatus(sprintf('At pick | err=%.1fmm', ikErr*1000));
end

function moveToPlaceCB(~,~)
    if isempty(placeTarget), setStatus('Set place target first'); return; end
    setStatus('Solving IK for place...');
    drawnow;
    [qSol, ikErr] = solveIK(placeTarget);
    set(hIKErrText, 'String', sprintf('IK: %.1f mm', ikErr*1000));
    animateToJoints(qSol);
    setStatus(sprintf('At place | err=%.1fmm', ikErr*1000));
end

function autoExecuteCB(~,~)
    % 一键自动: 确定Pick(最后箱子)/Place(下一个码垛位), 然后执行
    if isempty(sceneObjs)
        setStatus('Add a box to conveyor first!'); return;
    end
    maxBoxes = PALLET_COLS * PALLET_LAYERS;
    if placedBoxCount >= maxBoxes
        setStatus(sprintf('Pallet full (%d/%d)! Press Reset Count first', placedBoxCount, maxBoxes));
        return;
    end
    autoFillPick();
    autoFillPlace();
    executeCB([],[]);
end

function executeCB(~,~)
    if isempty(pickTarget) || isempty(placeTarget)
        setStatus('Error: Set both pick and place targets first'); return;
    end
    if isAnimating, setStatus('Already running'); return; end
    isAnimating = true;
    setStatus('Planning trajectory...');
    drawnow;

    [qPick, errPick]   = solveIK(pickTarget);
    [qPlace, errPlace] = solveIK(placeTarget);
    set(hIKErrText, 'String', sprintf('Pick=%.1f Place=%.1f mm', errPick*1000, errPlace*1000));

    q_home = [0, -pi/3, pi/4, 0, -pi/3, 0];
    qHovPk = qPick;  qHovPk(2)=qHovPk(2)+deg2rad(10); qHovPk(3)=qHovPk(3)-deg2rad(8);
    qHovPl = qPlace;  qHovPl(2)=qHovPl(2)+deg2rad(10); qHovPl(3)=qHovPl(3)-deg2rad(8);

    % 高位避障路径点: 保持J1, 将臂抬高至篮筐上方 (TCP Z > 2.1m)
    % J2=-35°(更直立), J3=15°(更伸展), J4=-85°, J5=-90° → TCP约2.3m
    q_high_pick  = qPick;
    q_high_pick(2)=deg2rad(-35); q_high_pick(3)=deg2rad(15);
    q_high_pick(4)=deg2rad(-85); q_high_pick(5)=-pi/2; q_high_pick(6)=0;
    q_high_place = qPlace;
    q_high_place(2)=deg2rad(-35); q_high_place(3)=deg2rad(15);
    q_high_place(4)=deg2rad(-85); q_high_place(5)=-pi/2; q_high_place(6)=0;

    clearTrailCB([],[]);

    % 找到当前pick的箱子, 用于动画中隐藏
    pickBoxIdx = findPickBox();

    waypoints = {q_home,'Home',false;
                 qHovPk,'Hover Pick',false;
                 qPick,'Pick',false;
                 qHovPk,'Lift',true;
                 q_high_pick,'Rise',true;        % 抬高到篮筐上方
                 q_high_place,'Transit',true;     % 高位转向码垛侧
                 qHovPl,'Hover Place',true;
                 qPlace,'Place',true;
                 qHovPl,'Retract',false;
                 q_high_place,'Rise Back',false;  % 抬高避障
                 q_home,'Home',false};

    for wi = 1:size(waypoints,1)
        if ~isAnimating, break; end
        wp_name = waypoints{wi,2};
        setStatus(sprintf('%s (%d/%d)', wp_name, wi, size(waypoints,1)));

        animateToJoints(waypoints{wi,1});

        % Pick: 移除传送带箱子, 创建跟随TCP的抓取箱
        if strcmp(wp_name, 'Pick') && ~isempty(pickBoxIdx)
            if pickBoxIdx <= length(sceneObjs)
                if isvalid(sceneObjs(pickBoxIdx).hPatch), delete(sceneObjs(pickBoxIdx).hPatch); end
                if isvalid(sceneObjs(pickBoxIdx).hLabel), delete(sceneObjs(pickBoxIdx).hLabel); end
                sceneObjs(pickBoxIdx) = [];
                if convBoxCount > 0, convBoxCount = convBoxCount - 1; end
                refreshObjList(); updatePalletInfo();
            end
            pickBoxIdx = [];
            % 创建跟随TCP的搬运箱 (hgtransform, 自动随TCP更新)
            deleteValid(hCarriedBox);
            htCB = hgtransform('Parent', ax3d);
            bsz = DEF_BOX_SZ;
            vb = [-bsz(1)/2 -bsz(2)/2 -bsz(3); bsz(1)/2 -bsz(2)/2 -bsz(3);
                   bsz(1)/2  bsz(2)/2 -bsz(3);-bsz(1)/2  bsz(2)/2 -bsz(3);
                  -bsz(1)/2 -bsz(2)/2 0;       bsz(1)/2 -bsz(2)/2 0;
                   bsz(1)/2  bsz(2)/2 0;      -bsz(1)/2  bsz(2)/2 0];
            patch('Parent',htCB,'Vertices',vb,...
                'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
                'FaceColor',[0.65 0.45 0.25],'EdgeColor',[0.3 0.3 0.3],'FaceAlpha',0.90,'LineWidth',1);
            Tcb = eye(4); Tcb(1:3,4) = lastTcp(:);
            set(htCB, 'Matrix', Tcb);
            hCarriedBox = htCB;
            drawnow limitrate;
        end
        % Place: 删除搬运箱, 在篮筐内放下 (placeTarget = 箱子顶部)
        if strcmp(wp_name, 'Place')
            deleteValid(hCarriedBox); hCarriedBox = gobjects(0);
            drawBox3D(ax3d, placeTarget(1)-DEF_BOX_SZ(1)/2, placeTarget(2)-DEF_BOX_SZ(2)/2, ...
                placeTarget(3)-DEF_BOX_SZ(3), DEF_BOX_SZ(1), DEF_BOX_SZ(2), DEF_BOX_SZ(3), ...
                [0.55 0.40 0.22]);
            text(ax3d, placeTarget(1), placeTarget(2), placeTarget(3)+0.04, ...
                sprintf('#%d', placedBoxCount+1), 'FontSize', 10, 'FontWeight', 'bold', ...
                'FontName', CJK_FONT, 'HorizontalAlignment', 'center', 'Color', [0 0.3 0.7]);
            placedBoxCount = placedBoxCount + 1;
            updatePalletInfo();
            drawnow;
        end
    end

    isAnimating = false;
    % 自动准备下一次
    maxBoxes = PALLET_COLS * PALLET_LAYERS;
    colMsg = '';
    if collisionCount > 0
        colMsg = sprintf(' | WARNING: %d collision points!', collisionCount);
    end
    if placedBoxCount < maxBoxes
        autoFillPlace();
        setStatus(sprintf('Cycle %d/%d done!%s Add next box or Auto Pick & Place', placedBoxCount, maxBoxes, colMsg));
    else
        setStatus(sprintf('Pallet FULL (%d/%d)!%s Press Reset Count', placedBoxCount, maxBoxes, colMsg));
    end
    if collisionCount > 0
        set(hIKErrText, 'String', sprintf('Pick=%.1f Place=%.1f mm | COL=%d', errPick*1000, errPlace*1000, collisionCount));
        set(hIKErrText, 'ForegroundColor', [0.8 0 0]);
    else
        set(hIKErrText, 'ForegroundColor', [0 0.5 0]);
        set(hIKErrText, 'String', sprintf('Pick=%.1f Place=%.1f mm | No collision', errPick*1000, errPlace*1000));
    end
end

function idx = findPickBox()
    % 找到距离pickTarget最近的可见Box (仅XY距离, 因Z有半高偏移)
    idx = [];
    if isempty(pickTarget), return; end
    bestD = inf;
    for ii = 1:length(sceneObjs)
        if ~strcmp(sceneObjs(ii).type, 'Box'), continue; end
        % 跳过已隐藏(已抓取)的箱子
        if isvalid(sceneObjs(ii).hPatch) && strcmp(get(sceneObjs(ii).hPatch,'Visible'),'off')
            continue;
        end
        d = norm(sceneObjs(ii).pos(1:2) - pickTarget(1:2));
        if d < bestD, bestD = d; idx = ii; end
    end
    if bestD > 0.5, idx = []; end  % XY距离太远则不匹配
end

function stopCB(~,~)
    isAnimating = false;
    setStatus('Stopped');
end

function clearTrailCB(~,~)
    trailXYZ = [];
    collisionCount = 0;
    deleteValid(hTrailLine);
    hTrailLine = gobjects(0);
    deleteValid(hCollisionPts);
    hCollisionPts = gobjects(0);
end

%% ========================================================================
%%  animateToJoints - 性能优化版: set()更新trail, 无重复FK
%% ========================================================================
function animateToJoints(qTarget)
    nSteps = max(3, round(get(hSpeedSlider, 'Value')));
    q_start = q_current;

    for si = 1:nSteps
        if ~isAnimating && si > 1, return; end  % 支持中途停止

        t = si / nSteps;
        q_current = (1-t) * q_start + t * qTarget;

        % 更新滑块 (批量, 不单独drawnow)
        for ji = 1:6
            set(hSliders(ji), 'Value', rad2deg(q_current(ji)));
            set(hSliderLabels(ji), 'String', sprintf('%+.1f', rad2deg(q_current(ji))));
        end

        % 更新机器人 (返回TCP, 无需再算FK)
        tcp = updateRobotDisplay();
        trailXYZ = [trailXYZ; tcp];

        % ---- 碰撞检测 ----
        [isCol, ~] = checkTcpCollision(tcp);
        if isCol
            collisionCount = collisionCount + 1;
            hc = plot3(ax3d, tcp(1), tcp(2), tcp(3), 'rx', ...
                'MarkerSize', 10, 'LineWidth', 2.5);
            if isempty(hCollisionPts) || ~all(isvalid(hCollisionPts))
                hCollisionPts = hc;
            else
                hCollisionPts(end+1) = hc;
            end
        end

        % 更新轨迹线
        if ~isempty(hTrailLine) && all(isvalid(hTrailLine)) && size(trailXYZ,1) > 1
            set(hTrailLine, 'XData', trailXYZ(:,1), 'YData', trailXYZ(:,2), 'ZData', trailXYZ(:,3));
        elseif size(trailXYZ,1) > 1
            deleteValid(hTrailLine);
            hTrailLine = plot3(ax3d, trailXYZ(:,1), trailXYZ(:,2), trailXYZ(:,3), ...
                '-', 'Color', [1 0.3 0 0.85], 'LineWidth', 2.5);
        end

        drawnow limitrate;
        pause(0.005);
    end
    q_current = qTarget;
    updateRobotDisplay();
    drawnow;
end

function onClose(~,~)
    isAnimating = false;
    delete(fig);
end

%% ========================================================================
%%  碰撞检测: TCP是否在障碍物体积内
%% ========================================================================
function [isCol, name] = checkTcpCollision(tcp)
    isCol = false; name = '';
    px = tcp(1); py = tcp(2); pz = tcp(3);
    margin = 0.04;

    % --- 篮筐立柱 (4根) ---
    f = defScene.frame; fcx = 0;
    fCorners = [fcx-f.wx/2 frameCY-f.dy/2;
                fcx+f.wx/2 frameCY-f.dy/2;
                fcx+f.wx/2 frameCY+f.dy/2;
                fcx-f.wx/2 frameCY+f.dy/2];
    postR = f.tubeR + margin;
    for ci = 1:4
        cx = fCorners(ci,1); cy = fCorners(ci,2);
        if abs(px-cx) < postR && abs(py-cy) < postR && pz > 0 && pz < f.h
            isCol = true; name = sprintf('Post%d', ci); return;
        end
    end

    % --- 篮筐横杆 (3面x4层) ---
    barR = f.tubeR*0.8 + margin;
    barLevels = [0.05, f.h/3, 2*f.h/3, f.h-0.05];
    fEdges = {[1,2],[2,3],[3,4],[4,1]};
    for lvi = 1:length(barLevels)
        zbar = barLevels(lvi);
        if abs(pz - zbar) > barR, continue; end
        for ei = 2:4
            i1 = fEdges{ei}(1); i2 = fEdges{ei}(2);
            x1b = fCorners(i1,1); y1b = fCorners(i1,2);
            x2b = fCorners(i2,1); y2b = fCorners(i2,2);
            dpt = ptSegDist2D(px,py, x1b,y1b, x2b,y2b);
            if dpt < barR
                isCol = true; name = sprintf('Bar L%d', lvi); return;
            end
        end
    end

    % --- 托盘 ---
    p = defScene.pallet;
    pxMin = fcx-p.wx/2; pxMax = fcx+p.wx/2;
    pyMin = frameCY-p.dy/2; pyMax = frameCY+p.dy/2;
    if px>pxMin+margin && px<pxMax-margin && py>pyMin+margin && py<pyMax-margin && pz>0 && pz<p.hz-margin
        isCol = true; name = 'Pallet'; return;
    end

    % --- 底座柜 ---
    cab = defScene.cab;
    if abs(px)<cab.wx/2-margin && abs(py)<cab.dy/2-margin && pz<cab.hz-margin && pz>0
        isCol = true; name = 'Cabinet'; return;
    end

    % --- 传送带结构 ---
    cv = defScene.conv;
    if px>convCX-cv.wx/2+margin && px<convCX+cv.wx/2-margin && ...
       py>convCY-cv.ly/2+margin && py<convCY+cv.ly/2-margin && pz<convSurfZ-margin && pz>0
        isCol = true; name = 'Conveyor'; return;
    end

    % --- 场景对象 ---
    for oi = 1:length(sceneObjs)
        o = sceneObjs(oi);
        if abs(px-o.pos(1))<o.sz(1)/2+margin && abs(py-o.pos(2))<o.sz(2)/2+margin && abs(pz-o.pos(3))<o.sz(3)/2+margin
            isCol = true; name = o.name; return;
        end
    end
end

function d = ptSegDist2D(px, py, x1, y1, x2, y2)
    dx = x2-x1; dy = y2-y1; len2 = dx*dx+dy*dy;
    if len2 < 1e-12, d = sqrt((px-x1)^2+(py-y1)^2); return; end
    t = max(0, min(1, ((px-x1)*dx+(py-y1)*dy)/len2));
    d = sqrt((px-x1-t*dx)^2 + (py-y1-t*dy)^2);
end

%% ========================================================================
%%                       静态场景绘图
%% ========================================================================
function drawStaticScene()
    patch(ax3d, 'Vertices',[-1.2 -1.5 0;1.5 -1.5 0;1.5 2.5 0;-1.2 2.5 0],'Faces',[1 2 3 4],...
          'FaceColor',[.92 .92 .90],'EdgeColor','none','FaceAlpha',0.4);
    c = defScene.cab;
    drawBox3D(ax3d, -c.wx/2, -c.dy/2, 0, c.wx, c.dy, c.hz, c.color);
    f = defScene.frame; fcx = 0;
    corners = [fcx-f.wx/2 frameCY-f.dy/2; fcx+f.wx/2 frameCY-f.dy/2;
               fcx+f.wx/2 frameCY+f.dy/2; fcx-f.wx/2 frameCY+f.dy/2];
    for ii=1:4
        drawTube(ax3d,corners(ii,1),corners(ii,2),0,corners(ii,1),corners(ii,2),f.h,f.tubeR,f.color,CYL_N);
    end
    edges={[1,2],[2,3],[3,4],[4,1]};
    for hz_lev=[0.05 f.h/3 2*f.h/3 f.h-0.05]
        for ei=2:4
            i1=edges{ei}(1); i2=edges{ei}(2);
            drawTube(ax3d,corners(i1,1),corners(i1,2),hz_lev,corners(i2,1),corners(i2,2),hz_lev,f.tubeR*0.8,f.color,CYL_N);
        end
    end
    px=fcx-defScene.pallet.wx/2; py=frameCY-defScene.pallet.dy/2;
    drawBox3D(ax3d, px, py, 0, defScene.pallet.wx, defScene.pallet.dy, defScene.pallet.hz, defScene.pallet.color);
    text(ax3d, fcx, frameCY, defScene.pallet.hz+0.03, sprintf('Pallet %dcm',round(defScene.pallet.hz*100)), ...
        'FontSize',11,'HorizontalAlignment','center','FontWeight','bold','FontName',CJK_FONT,'Color',[0.05 0.15 0.45]);
    cv = defScene.conv; ly=cv.ly; wx=cv.wx; hz=cv.hz;
    x0=convCX-wx/2; y0=convCY-ly/2;
    drawBox3D(ax3d,x0-0.015,y0,hz-0.06,0.015,ly,0.06,[0.4 0.4 0.42]);
    drawBox3D(ax3d,x0+wx,y0,hz-0.06,0.015,ly,0.06,[0.4 0.4 0.42]);
    lw=0.035; yL=[y0+0.2 convCY y0+ly-0.2];
    for yi=1:3, for s=[-1 1]
        lx=convCX+s*(wx/2-0.06);
        drawBox3D(ax3d,lx-lw/2,yL(yi)-lw/2,0,lw,lw,hz-0.01,[0.25 0.25 0.25]);
    end; end
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        [X2,Y2,Z2]=cylinder(cv.rollerR,CYL_N); Z2=Z2*wx*0.9-wx*0.9/2;
        surf(ax3d,Z2+convCX,zeros(size(X2))+ry,X2+hz+cv.rollerR,...
            'FaceColor',[0.55 0.55 0.55],'EdgeColor','none','FaceAlpha',0.6);
    end
    bz=hz+cv.rollerR;
    bV=[x0+0.02 y0+0.04 bz;x0+wx-0.02 y0+0.04 bz;x0+wx-0.02 y0+ly-0.04 bz;x0+0.02 y0+ly-0.04 bz;
        x0+0.02 y0+0.04 bz+cv.beltH;x0+wx-0.02 y0+0.04 bz+cv.beltH;
        x0+wx-0.02 y0+ly-0.04 bz+cv.beltH;x0+0.02 y0+ly-0.04 bz+cv.beltH];
    patch(ax3d,'Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[0.15 0.15 0.15],'FaceAlpha',0.92);
    text(ax3d, convCX, convCY, hz+cv.rollerR+cv.beltH+0.05, 'Conveyor', ...
        'FontSize',11,'HorizontalAlignment','center','FontWeight','bold','FontName',CJK_FONT,'Color',[0.2 0.2 0.3]);
end

%% ========================================================================
%%                       工具函数
%% ========================================================================
function setStatus(msg)
    set(hStatusText, 'String', ['  ' msg ' | v2.0']);
    drawnow limitrate;
end

function deleteValid(h)
    if ~isempty(h), v = isvalid(h); if any(v), delete(h(v)); end; end
end

function h = drawBox3D(ax, x,y,z,dx,dy,dz,col)
    v=[x y z;x+dx y z;x+dx y+dy z;x y+dy z;x y z+dz;x+dx y z+dz;x+dx y+dy z+dz;x y+dy z+dz];
    h = patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',col,'EdgeColor',[0.3 0.3 0.3],'FaceAlpha',0.95,'LineWidth',1);
end

function [X,Y,Z] = capsuleMesh(radius, L, nSeg)
    nHemi = max(3, floor(nSeg/2));
    tBot = linspace(-pi/2, 0, nHemi+1); rBot = radius*cos(tBot); zBot = radius*sin(tBot)-L/2;
    rCyl = [radius; radius]; zCyl = [-L/2; L/2];
    tTop = linspace(0, pi/2, nHemi+1); rTop = radius*cos(tTop); zTop = radius*sin(tTop)+L/2;
    rAll = [rBot(:); rCyl(:); rTop(:)]; zAll = [zBot(:); zCyl(:); zTop(:)];
    theta = linspace(0, 2*pi, nSeg+1);
    X = rAll * cos(theta); Y = rAll * sin(theta); Z = repmat(zAll, 1, nSeg+1);
end

function R = rotZtoDirection(d)
    d = d(:)/norm(d); z=[0;0;1]; v=cross(z,d); s=norm(v); c=dot(z,d);
    if s<1e-10, if c>0, R=eye(3); else, R=diag([-1,-1,1]); end; return; end
    vx=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    R = eye(3) + vx + vx*vx*(1-c)/(s*s);
end

function drawTube(ax, x1,y1,z1,x2,y2,z2,radius,color,cylN)
    [X2,Y2,Z2]=cylinder(radius,cylN);
    v_dir=[x2-x1;y2-y1;z2-z1]; l=norm(v_dir);
    if l<0.001, return; end
    Z2=Z2*l; dd=[0;0;1]; td=v_dir/l; cp=cross(dd,td);
    if norm(cp)>1e-6
        ax_ang=acos(max(-1,min(1,dot(dd,td)))); cpn=cp/norm(cp);
        ca=cos(ax_ang); sa=sin(ax_ang); ta=1-ca;
        RR=[ta*cpn(1)^2+ca ta*cpn(1)*cpn(2)-sa*cpn(3) ta*cpn(1)*cpn(3)+sa*cpn(2);
            ta*cpn(1)*cpn(2)+sa*cpn(3) ta*cpn(2)^2+ca ta*cpn(2)*cpn(3)-sa*cpn(1);
            ta*cpn(1)*cpn(3)-sa*cpn(2) ta*cpn(2)*cpn(3)+sa*cpn(1) ta*cpn(3)^2+ca];
    else
        RR=eye(3); if dot(dd,td)<0, RR(3,3)=-1; RR(1,1)=-1; end
    end
    for ii=1:numel(X2)
        pt=RR*[X2(ii);Y2(ii);Z2(ii)]; X2(ii)=pt(1)+x1; Y2(ii)=pt(2)+y1; Z2(ii)=pt(3)+z1;
    end
    surf(ax,X2,Y2,Z2,'FaceColor',color,'EdgeColor','none','FaceAlpha',0.85);
end

function fontName = selectFont()
    fontName = 'Noto Sans CJK SC';
    allF = listfonts();
    if any(strcmp(allF, fontName)), return; end
    for c = {'Noto Serif CJK SC','SimHei','WenQuanYi Micro Hei','Arial Unicode MS'}
        if any(strcmp(allF, c{1})), fontName = c{1}; return; end
    end
    fontName = get(0, 'DefaultAxesFontName');
end

end  % PalletizingSimApp
