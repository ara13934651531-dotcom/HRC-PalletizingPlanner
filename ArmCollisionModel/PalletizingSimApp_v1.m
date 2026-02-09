function PalletizingSimApp()
%% PalletizingSimApp - HR_S50-2000 交互式码垛仿真平台
%
%  功能:
%    1. 3D场景: hgtransform 胶囊体机器人模型, 实时渲染 (>15 fps)
%    2. 场景编辑: 添加/删除/移动 箱子和障碍物, 自定义尺寸颜色
%    3. 关节控制: 6轴滑块实时调节, TCP坐标同步显示
%    4. 取放工作流: 设定Pick/Place目标, IK求解, 轨迹动画
%    5. 交互操作: 鼠标旋转/缩放视角, 对象选择, 实时状态反馈
%
%  启动:
%    >> PalletizingSimApp()
%
%  类似 Simulink 的交互模式:
%    - 右侧面板自由编辑场景 (添加箱子/圆柱/长方体障碍物)
%    - 拖动关节滑块直接控制机器人姿态
%    - 设定 Pick/Place 目标后一键规划执行
%    - 轨迹实时显示, 完成后可清除
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

%% ========================================================================
%%                         共享状态变量
%% ========================================================================
q_current = [0, -pi/3, pi/4, 0, -pi/3, 0];  % 当前关节角 (rad)
baseZ = 0.80;
Tb = eye(4); Tb(3,4) = baseZ;

% DH参数 (m, 与FK_SSerial一致)
DH.d1=0.2965; DH.d2=0.3362; DH.d3=0.239;
DH.d4=0.1585; DH.d5=0.1585; DH.d6=0.1345;
DH.a2=-0.900; DH.a3=-0.9415;

% 关节限位 (deg)
jLimDeg = [-360 360; -190 10; -165 165; -360 360; -360 360; -360 360];

% 碰撞模型几何 (m, 从JSON)
capsuleDefs = {
    'base',     [0 0 0.03],     [0 0 0.3362],   0.13, 1;   % 连接到T00
    'lowerArm', [0 0 0.28],     [0.900 0 0.28],  0.13, 3;   % 连接到T02
    'elbow',    [-0.02 0 0.08], [0.9415 0 0.08], 0.10, 4;   % 连接到T03
    'upperArm', [0 0 -0.06],   [0 0 0.12],      0.06, 5;   % 连接到T04
};
sphereDefs = {
    'wrist', [0 0 0.03], 0.12, 6;  % 连接到T05
};

% 场景对象
sceneObjs = struct('name',{},'type',{},'pos',{},'sz',{},'color',{},'hPatch',{});

% 任务
pickTarget  = [];
placeTarget = [];
trailXYZ = [];
isAnimating = false;

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

% 渲染参数
CYL_N = 8;            % 圆柱/管道圆周面数
CAPSULE_N = 10;       % 胶囊体圆周面数
CAPSULE_ALPHA = 0.35; % 透明度
CAPSULE_COLORS = {[0.75 0.75 0.80],[0.50 0.55 0.85],[0.85 0.50 0.45],...
                  [0.50 0.80 0.55],[0.80 0.75 0.50]};

% 字体
CJK_FONT = selectFont();

%% ========================================================================
%%                        图形句柄 (预分配)
%% ========================================================================
hCapsuleHT = gobjects(0);   % hgtransform for capsules
capsuleLocalT = {};          % local offset 4x4 matrix
capsuleLinkIdx = [];         % which FK frame each capsule is attached to
nCapsules = 0;

hSphereHT = gobjects(0);    % hgtransform for spheres
sphereLocalT = {};
sphereLinkIdx = [];
nSpheres = 0;

hJointMarkers = gobjects(0); % 关节标记
hTcpMarker    = gobjects(0); % TCP标记
hTrailLine    = gobjects(0); % 轨迹线
hPickMark     = gobjects(0); % Pick目标标记
hPlaceMark    = gobjects(0); % Place目标标记
hCarryBox     = gobjects(0); % 携带箱子

% UI句柄
hSliders  = gobjects(6,1);
hSliderLabels = gobjects(6,1);
hTcpText  = gobjects(0);
hStatusText = gobjects(0);

%% ========================================================================
%%                        创建主窗口
%% ========================================================================
fig = figure('Name', 'HR_S50-2000 Interactive Palletizing Simulator', ...
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
xlim(ax3d,[-1.5 2.0]); ylim(ax3d,[-2.0 3.0]); zlim(ax3d,[0 2.5]);
view(ax3d,135,25);
camlight('headlight'); lighting(ax3d,'gouraud');
title(ax3d, 'HR\_S50-2000 Interactive Simulator', ...
    'FontSize', 16, 'FontWeight', 'bold', 'FontName', CJK_FONT, 'Color', [0.1 0.1 0.3]);
rotate3d(ax3d, 'on');

% ===== 右侧面板 =====
panelX = 0.635; panelW = 0.355;

% --- 场景编辑面板 ---
scnPnl = uipanel(fig, 'Title', 'Scene Editor', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', CJK_FONT, ...
    'Position', [panelX 0.68 panelW 0.31], ...
    'BackgroundColor', [0.96 0.96 0.98]);

yy = 0.85; dy = 0.13; lw = 0.40;
% 对象类型
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

yy = yy - dy;
% 尺寸
uicontrol(scnPnl,'Style','text','String','Size(m):','Units','normalized',...
    'Position',[0.02 yy 0.14 0.10],'FontSize',10,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hSzX = uicontrol(scnPnl,'Style','edit','String','0.40','Units','normalized','Position',[0.17 yy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hSzY = uicontrol(scnPnl,'Style','edit','String','0.30','Units','normalized','Position',[0.30 yy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hSzZ = uicontrol(scnPnl,'Style','edit','String','0.25','Units','normalized','Position',[0.43 yy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
% 位置
uicontrol(scnPnl,'Style','text','String','Pos(m):','Units','normalized',...
    'Position',[0.02 yy-dy 0.14 0.10],'FontSize',10,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hPosX = uicontrol(scnPnl,'Style','edit','String',num2str(convCX,'%.2f'),'Units','normalized','Position',[0.17 yy-dy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hPosY = uicontrol(scnPnl,'Style','edit','String','0.20','Units','normalized','Position',[0.30 yy-dy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);
hPosZ = uicontrol(scnPnl,'Style','edit','String',num2str(convSurfZ+0.125,'%.3f'),'Units','normalized','Position',[0.43 yy-dy+0.01 0.12 0.10],'FontSize',10,'FontName',CJK_FONT);

% 对象列表
hObjList = uicontrol(scnPnl,'Style','listbox','String',{},'Units','normalized',...
    'Position',[0.57 yy-dy+0.01 0.25 0.22],'FontSize',10,'FontName',CJK_FONT);
uicontrol(scnPnl,'Style','pushbutton','String','Del','Units','normalized',...
    'Position',[0.83 yy-0.02 0.15 0.10],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'Callback',@delObjectCB,'BackgroundColor',[0.8 0.3 0.3],'ForegroundColor','w');
uicontrol(scnPnl,'Style','pushbutton','String','Clear','Units','normalized',...
    'Position',[0.83 yy-dy+0.01 0.15 0.10],'FontSize',11,'FontWeight','bold',...
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
        'String',sprintf('%+.1f°',rad2deg(q_current(ji))),...
        'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
        'BackgroundColor',[0.96 0.96 0.98]);
    % Java listener for continuous update
    try
        addlistener(hSliders(ji), 'ContinuousValueChange', @jointSliderCB);
    catch
    end
end
hTcpText = uicontrol(robPnl,'Style','text','Units','normalized',...
    'Position',[0.02 0.01 0.55 0.08],...
    'String','TCP: X=0 Y=0 Z=0 mm','FontSize',11,'FontWeight','bold',...
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
    'Position', [panelX 0.06 panelW 0.27], ...
    'BackgroundColor', [0.96 0.96 0.98]);

% Pick/Place目标输入
uicontrol(taskPnl,'Style','text','String','Pick(m):','Units','normalized',...
    'Position',[0.02 0.82 0.14 0.12],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hPickX = uicontrol(taskPnl,'Style','edit','String',num2str(convCX,'%.2f'),'Units','normalized','Position',[0.17 0.84 0.11 0.12],'FontSize',10,'FontName',CJK_FONT);
hPickY = uicontrol(taskPnl,'Style','edit','String','0.20','Units','normalized','Position',[0.29 0.84 0.11 0.12],'FontSize',10,'FontName',CJK_FONT);
hPickZ = uicontrol(taskPnl,'Style','edit','String',num2str(convSurfZ+0.25,'%.3f'),'Units','normalized','Position',[0.41 0.84 0.11 0.12],'FontSize',10,'FontName',CJK_FONT);
uicontrol(taskPnl,'Style','pushbutton','String','Set Pick','Units','normalized',...
    'Position',[0.54 0.83 0.20 0.14],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@setPickCB,'BackgroundColor',[0.2 0.7 0.3],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','Use TCP','Units','normalized',...
    'Position',[0.76 0.83 0.22 0.14],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@usePickTcpCB,'BackgroundColor',[0.4 0.6 0.3],'ForegroundColor','w');

uicontrol(taskPnl,'Style','text','String','Place(m):','Units','normalized',...
    'Position',[0.02 0.64 0.14 0.12],'FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hPlaceX = uicontrol(taskPnl,'Style','edit','String','0.00','Units','normalized','Position',[0.17 0.66 0.11 0.12],'FontSize',10,'FontName',CJK_FONT);
hPlaceY = uicontrol(taskPnl,'Style','edit','String',num2str(frameCY-0.30,'%.2f'),'Units','normalized','Position',[0.29 0.66 0.11 0.12],'FontSize',10,'FontName',CJK_FONT);
hPlaceZ = uicontrol(taskPnl,'Style','edit','String',num2str(defScene.pallet.hz+0.125,'%.3f'),'Units','normalized','Position',[0.41 0.66 0.11 0.12],'FontSize',10,'FontName',CJK_FONT);
uicontrol(taskPnl,'Style','pushbutton','String','Set Place','Units','normalized',...
    'Position',[0.54 0.65 0.20 0.14],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@setPlaceCB,'BackgroundColor',[0.7 0.3 0.2],'ForegroundColor','w');
uicontrol(taskPnl,'Style','pushbutton','String','Use TCP','Units','normalized',...
    'Position',[0.76 0.65 0.22 0.14],'FontSize',10,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@usePlaceTcpCB,'BackgroundColor',[0.6 0.4 0.3],'ForegroundColor','w');

% 执行按钮行
uicontrol(taskPnl,'Style','pushbutton','String','Plan & Execute','Units','normalized',...
    'Position',[0.02 0.40 0.30 0.18],'FontSize',12,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@executeCB,'BackgroundColor',[0.15 0.55 0.80],'ForegroundColor','w');

uicontrol(taskPnl,'Style','text','String','Speed:','Units','normalized',...
    'Position',[0.34 0.43 0.10 0.10],'FontSize',10,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
hSpeedSlider = uicontrol(taskPnl,'Style','slider','Units','normalized',...
    'Position',[0.45 0.45 0.25 0.06],'Min',1,'Max',50,'Value',15);
hSpeedLabel = uicontrol(taskPnl,'Style','text','Units','normalized',...
    'Position',[0.71 0.43 0.10 0.10],'String','15','FontSize',10,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98]);
try addlistener(hSpeedSlider,'ContinuousValueChange',@(s,~) set(hSpeedLabel,'String',num2str(round(s.Value)))); catch; end

uicontrol(taskPnl,'Style','pushbutton','String','Stop','Units','normalized',...
    'Position',[0.83 0.40 0.15 0.18],'FontSize',12,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@stopCB,'BackgroundColor',[0.8 0.2 0.2],'ForegroundColor','w');

% 底行
uicontrol(taskPnl,'Style','pushbutton','String','Clear Trail','Units','normalized',...
    'Position',[0.02 0.15 0.25 0.16],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@clearTrailCB,'BackgroundColor',[0.6 0.5 0.3],'ForegroundColor','w');

uicontrol(taskPnl,'Style','pushbutton','String','Move to Pick','Units','normalized',...
    'Position',[0.29 0.15 0.27 0.16],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@moveToPickCB,'BackgroundColor',[0.3 0.6 0.4],'ForegroundColor','w');

uicontrol(taskPnl,'Style','pushbutton','String','Move to Place','Units','normalized',...
    'Position',[0.58 0.15 0.27 0.16],'FontSize',11,'FontWeight','bold','FontName',CJK_FONT,...
    'Callback',@moveToPlaceCB,'BackgroundColor',[0.6 0.3 0.4],'ForegroundColor','w');

hIKErrText = uicontrol(taskPnl,'Style','text','Units','normalized',...
    'Position',[0.02 0.01 0.55 0.10],...
    'String','IK Error: -- mm','FontSize',11,'FontWeight','bold',...
    'FontName',CJK_FONT,'BackgroundColor',[0.96 0.96 0.98],'ForegroundColor',[0.5 0.1 0.1]);

% --- 状态栏 ---
hStatusText = uicontrol(fig,'Style','text','Units','normalized',...
    'Position',[0.01 0.001 0.98 0.03],...
    'String','Ready | HR_S50-2000 Interactive Palletizing Simulator v1.0','FontSize',11,...
    'FontWeight','bold','FontName',CJK_FONT,...
    'BackgroundColor',[0.15 0.35 0.60],'ForegroundColor','w',...
    'HorizontalAlignment','center');

%% ========================================================================
%%                     初始化: 静态场景 + 机器人
%% ========================================================================
setStatus('Initializing scene...');
drawStaticScene();
initRobotModel();
updateRobotDisplay();
setStatus('Ready — Drag joint sliders or add scene objects');

%% ========================================================================
%%                       正运动学 (独立, 无全局变量)
%% ========================================================================
function T_all = FK_local(qin)
    % 标准DH, 与FK_SSerial完全一致
    % 返回 cell{7}: T00, T01, T02, T03, T04, T05, T0T
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
    T00=eye(4);
    T01=Tlinks{1};
    T02=T01*Tlinks{2};
    T03=T02*Tlinks{3};
    T04=T03*Tlinks{4};
    T05=T04*Tlinks{5};
    T06=T05*Tlinks{6};
    T0T=T06; % 无工具: T6T=eye(4)
    T_all = {T00, T01, T02, T03, T04, T05, T0T};
end

%% ========================================================================
%%                       IK搜索 (独立)
%% ========================================================================
function [qSol, ikErr] = solveIK(targetXYZ)
    % 搜索IK: 输入世界坐标(m), 输出6关节角(rad)+误差(m)
    tx = targetXYZ(1); ty = targetXYZ(2); tz = targetXYZ(3);
    dx = tx; dy = ty; % base at (0,0,baseZ)
    q1 = atan2(-dy, -dx);
    tr = sqrt(dx^2+dy^2); tzr = tz - baseZ;
    eb=inf; q2b=-pi/2; q3b=pi/2; q4b=-pi/2;
    % 粗搜索
    for q2d=-175:3:-30
        for q3d=10:3:165
            for q4d=[-150,-135,-120,-90,-60,-45]
                q2=deg2rad(q2d); q3=deg2rad(q3d); q4=deg2rad(q4d);
                T_all=FK_local([q1,q2,q3,q4,-pi/2,0]);
                T=T_all{7}; % T0T
                r_fk=sqrt(T(1,4)^2+T(2,4)^2); z_fk=T(3,4);
                pe=sqrt((r_fk-tr)^2+(z_fk-tzr)^2);
                ez=T(1:3,3); zd=acos(max(-1,min(1,dot(ez,[0;0;-1]))));
                tot=pe+0.08*zd;
                if tot<eb && zd<30*pi/180, eb=pe; q2b=q2; q3b=q3; q4b=q4; end
            end
        end
    end
    % 精细搜索
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
%%                     机器人模型初始化 (hgtransform + 胶囊体)
%% ========================================================================
function initRobotModel()
    nCapsules = size(capsuleDefs, 1);
    hCapsuleHT = gobjects(nCapsules, 1);
    capsuleLocalT = cell(nCapsules, 1);
    capsuleLinkIdx = zeros(nCapsules, 1);

    for ci = 1:nCapsules
        p1 = capsuleDefs{ci,2};
        p2 = capsuleDefs{ci,3};
        radius = capsuleDefs{ci,4};
        linkId = capsuleDefs{ci,5};
        capsuleLinkIdx(ci) = linkId;

        % 创建 hgtransform
        ht = hgtransform('Parent', ax3d);
        hCapsuleHT(ci) = ht;

        % 创建胶囊体网格 (沿Z轴, 中心在原点)
        L = norm(p2 - p1);
        [Xc,Yc,Zc] = capsuleMesh(radius, L, CAPSULE_N);
        col = CAPSULE_COLORS{min(ci, length(CAPSULE_COLORS))};
        surf(Xc, Yc, Zc, 'Parent', ht, 'FaceColor', col, 'EdgeColor', 'none', ...
            'FaceAlpha', CAPSULE_ALPHA, 'FaceLighting', 'gouraud');

        % 计算local offset: 将Z轴胶囊映射到 p1→p2
        mid = (p1 + p2) / 2;
        if L > 1e-6
            d_vec = (p2 - p1) / L;
            R = rotZtoDirection(d_vec);
        else
            R = eye(3);
        end
        Tlocal = eye(4);
        Tlocal(1:3,1:3) = R;
        Tlocal(1:3,4) = mid(:);
        capsuleLocalT{ci} = Tlocal;
    end

    % 球体 (腕关节)
    nSpheres = size(sphereDefs, 1);
    hSphereHT = gobjects(nSpheres, 1);
    sphereLocalT = cell(nSpheres, 1);
    sphereLinkIdx = zeros(nSpheres, 1);

    for si = 1:nSpheres
        offset = sphereDefs{si, 2};
        radius = sphereDefs{si, 3};
        linkId = sphereDefs{si, 4};
        sphereLinkIdx(si) = linkId;

        ht = hgtransform('Parent', ax3d);
        hSphereHT(si) = ht;

        [Xs,Ys,Zs] = sphere(CAPSULE_N);
        Xs = Xs*radius; Ys = Ys*radius; Zs = Zs*radius;
        surf(Xs, Ys, Zs, 'Parent', ht, 'FaceColor', [0.80 0.75 0.50], ...
            'EdgeColor', 'none', 'FaceAlpha', CAPSULE_ALPHA, 'FaceLighting', 'gouraud');

        Tlocal = eye(4);
        Tlocal(1:3,4) = offset(:);
        sphereLocalT{si} = Tlocal;
    end

    % TCP标记
    hTcpMarker = plot3(ax3d, 0, 0, baseZ, 'rp', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'r', 'LineWidth', 1.5);

    % 关节位置标记
    hJointMarkers = scatter3(ax3d, zeros(8,1), zeros(8,1), zeros(8,1), ...
        50, 'k', 'filled', 'MarkerEdgeColor', [0.3 0.3 0.3]);
end

%% ========================================================================
%%                   更新机器人显示 (只改 hgtransform.Matrix)
%% ========================================================================
function updateRobotDisplay()
    T_all = FK_local(q_current);

    % 世界坐标
    Tw = cell(7,1);
    for ii = 1:7
        Tw{ii} = Tb * T_all{ii};
    end

    % 更新胶囊体
    for ci = 1:nCapsules
        li = capsuleLinkIdx(ci);
        worldT = Tw{li} * capsuleLocalT{ci};
        set(hCapsuleHT(ci), 'Matrix', worldT);
    end

    % 更新球体
    for si = 1:nSpheres
        li = sphereLinkIdx(si);
        worldT = Tw{li} * sphereLocalT{si};
        set(hSphereHT(si), 'Matrix', worldT);
    end

    % 更新TCP标记
    tcp = Tw{7}(1:3,4)';
    set(hTcpMarker, 'XData', tcp(1), 'YData', tcp(2), 'ZData', tcp(3));

    % 更新关节标记
    jp = zeros(8,3);
    jp(1,:) = [0, 0, baseZ];
    for ii = 1:7
        jp(ii+1,:) = Tw{ii}(1:3,4)';
    end
    set(hJointMarkers, 'XData', jp(:,1), 'YData', jp(:,2), 'ZData', jp(:,3));

    % TCP信息
    set(hTcpText, 'String', sprintf('TCP: X=%+.0f Y=%+.0f Z=%+.0f mm', ...
        tcp(1)*1000, tcp(2)*1000, tcp(3)*1000));

    drawnow limitrate;
end

%% ========================================================================
%%                         回调函数
%% ========================================================================
function jointSliderCB(~,~)
    for ji = 1:6
        q_current(ji) = deg2rad(hSliders(ji).Value);
        set(hSliderLabels(ji), 'String', sprintf('%+.1f°', hSliders(ji).Value));
    end
    updateRobotDisplay();
end

function homePoseCB(~,~)
    setJoints([0, -60, 45, 0, -60, 0]);
end

function zeroPoseCB(~,~)
    setJoints([0, 0, 0, 0, 0, 0]);
end

function setJoints(q_deg)
    for ji = 1:6
        q_current(ji) = deg2rad(q_deg(ji));
        set(hSliders(ji), 'Value', q_deg(ji));
        set(hSliderLabels(ji), 'String', sprintf('%+.1f°', q_deg(ji)));
    end
    updateRobotDisplay();
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
    colors = {[0.65 0.45 0.25],[0.55 0.55 0.60],[0.45 0.70 0.45],[0.70 0.45 0.65],[0.50 0.60 0.75]};
    cidx = mod(length(sceneObjs), length(colors)) + 1;

    obj.name = nm;
    obj.type = tp;
    obj.pos = [px py pz];
    obj.sz = [sx sy sz];
    obj.color = colors{cidx};

    switch tp
        case 'Box'
            obj.hPatch = drawBox3D(ax3d, px-sx/2, py-sy/2, pz-sz/2, sx, sy, sz, obj.color);
        case 'Cylinder'
            [Xc,Yc,Zc] = cylinder(sx/2, 16);
            Zc = Zc * sz;
            obj.hPatch = surf(ax3d, Xc+px, Yc+py, Zc+pz-sz/2, 'FaceColor', obj.color, ...
                'EdgeColor', 'none', 'FaceAlpha', 0.9);
        case 'Wall'
            obj.hPatch = drawBox3D(ax3d, px-sx/2, py-sy/2, 0, sx, sy, sz, [0.7 0.7 0.7]);
    end

    % 标签
    text(ax3d, px, py, pz+sz/2+0.06, nm, 'FontSize', 10, 'FontWeight', 'bold', ...
        'FontName', CJK_FONT, 'HorizontalAlignment', 'center', 'Color', [0.2 0.2 0.5],...
        'Tag', ['label_' nm]);

    sceneObjs(end+1) = obj;
    refreshObjList();
    % 更新默认名称
    hObjName.String = sprintf('%s%d', tp, length(sceneObjs)+1);
    setStatus(sprintf('Added %s "%s" at [%.2f, %.2f, %.2f]', tp, nm, px, py, pz));
end

function delObjectCB(~,~)
    idx = hObjList.Value;
    if isempty(sceneObjs) || idx < 1 || idx > length(sceneObjs), return; end
    nm = sceneObjs(idx).name;
    if isvalid(sceneObjs(idx).hPatch), delete(sceneObjs(idx).hPatch); end
    % 删除标签
    lbl = findobj(ax3d, 'Tag', ['label_' nm]);
    if ~isempty(lbl), delete(lbl); end
    sceneObjs(idx) = [];
    if idx > length(sceneObjs), hObjList.Value = max(1, length(sceneObjs)); end
    refreshObjList();
    setStatus(sprintf('Deleted "%s"', nm));
end

function clearSceneCB(~,~)
    for ii = 1:length(sceneObjs)
        if isvalid(sceneObjs(ii).hPatch), delete(sceneObjs(ii).hPatch); end
        lbl = findobj(ax3d, 'Tag', ['label_' sceneObjs(ii).name]);
        if ~isempty(lbl), delete(lbl); end
    end
    sceneObjs = struct('name',{},'type',{},'pos',{},'sz',{},'color',{},'hPatch',{});
    refreshObjList();
    setStatus('Scene cleared');
end

function refreshObjList()
    names = {};
    for ii = 1:length(sceneObjs)
        names{end+1} = sprintf('%s [%s]', sceneObjs(ii).name, sceneObjs(ii).type);
    end
    hObjList.String = names;
end

function setPickCB(~,~)
    pickTarget = [str2double(hPickX.String), str2double(hPickY.String), str2double(hPickZ.String)];
    if any(isnan(pickTarget)), setStatus('Error: Invalid pick coordinates'); pickTarget=[]; return; end
    % 显示标记
    deleteValid(hPickMark);
    hPickMark = plot3(ax3d, pickTarget(1), pickTarget(2), pickTarget(3), ...
        'gv', 'MarkerSize', 16, 'LineWidth', 3, 'MarkerFaceColor', [0 0.8 0]);
    setStatus(sprintf('Pick target set: [%.3f, %.3f, %.3f] m', pickTarget));
end

function setPlaceCB(~,~)
    placeTarget = [str2double(hPlaceX.String), str2double(hPlaceY.String), str2double(hPlaceZ.String)];
    if any(isnan(placeTarget)), setStatus('Error: Invalid place coordinates'); placeTarget=[]; return; end
    deleteValid(hPlaceMark);
    hPlaceMark = plot3(ax3d, placeTarget(1), placeTarget(2), placeTarget(3), ...
        'rv', 'MarkerSize', 16, 'LineWidth', 3, 'MarkerFaceColor', [0.8 0 0]);
    setStatus(sprintf('Place target set: [%.3f, %.3f, %.3f] m', placeTarget));
end

function usePickTcpCB(~,~)
    T_all = FK_local(q_current);
    tcp = Tb * T_all{7};
    hPickX.String = sprintf('%.3f', tcp(1,4));
    hPickY.String = sprintf('%.3f', tcp(2,4));
    hPickZ.String = sprintf('%.3f', tcp(3,4));
    setPickCB([],[]);
end

function usePlaceTcpCB(~,~)
    T_all = FK_local(q_current);
    tcp = Tb * T_all{7};
    hPlaceX.String = sprintf('%.3f', tcp(1,4));
    hPlaceY.String = sprintf('%.3f', tcp(2,4));
    hPlaceZ.String = sprintf('%.3f', tcp(3,4));
    setPlaceCB([],[]);
end

function moveToPickCB(~,~)
    if isempty(pickTarget)
        setStatus('Error: Set pick target first'); return;
    end
    setStatus('Solving IK for pick target...');
    drawnow;
    [qSol, ikErr] = solveIK(pickTarget);
    set(hIKErrText, 'String', sprintf('IK Error: %.1f mm', ikErr*1000));
    if ikErr > 0.05
        setStatus(sprintf('Warning: IK error %.1f mm (>50mm), target may be unreachable', ikErr*1000));
    end
    animateToJoints(qSol);
    setStatus(sprintf('Moved to pick | IK err=%.1fmm', ikErr*1000));
end

function moveToPlaceCB(~,~)
    if isempty(placeTarget)
        setStatus('Error: Set place target first'); return;
    end
    setStatus('Solving IK for place target...');
    drawnow;
    [qSol, ikErr] = solveIK(placeTarget);
    set(hIKErrText, 'String', sprintf('IK Error: %.1f mm', ikErr*1000));
    if ikErr > 0.05
        setStatus(sprintf('Warning: IK error %.1f mm (>50mm)', ikErr*1000));
    end
    animateToJoints(qSol);
    setStatus(sprintf('Moved to place | IK err=%.1fmm', ikErr*1000));
end

function executeCB(~,~)
    if isempty(pickTarget) || isempty(placeTarget)
        setStatus('Error: Set both pick and place targets first');
        return;
    end
    if isAnimating
        setStatus('Animation already running'); return;
    end
    isAnimating = true;
    setStatus('Planning pick-place trajectory...');
    drawnow;

    % IK求解
    [qPick, errPick]   = solveIK(pickTarget);
    [qPlace, errPlace] = solveIK(placeTarget);
    set(hIKErrText, 'String', sprintf('Pick err=%.1f Place err=%.1f mm', errPick*1000, errPlace*1000));

    q_home = [0, -pi/3, pi/4, 0, -pi/3, 0];
    q_safe = [deg2rad(-90), deg2rad(-55), deg2rad(40), deg2rad(-75), -pi/2, 0];

    % Hover姿态
    qHovPk = qPick; qHovPk(2)=qHovPk(2)+deg2rad(10); qHovPk(3)=qHovPk(3)-deg2rad(8);
    qHovPl = qPlace; qHovPl(2)=qHovPl(2)+deg2rad(10); qHovPl(3)=qHovPl(3)-deg2rad(8);

    % 清除旧轨迹
    clearTrailCB([],[]);

    % 执行完整pick-place序列
    waypoints = {q_home,'Home'; qHovPk,'Hover Pick'; qPick,'Pick'; qHovPk,'Lift';
                 q_safe,'Safe Transit'; qHovPl,'Hover Place'; qPlace,'Place';
                 qHovPl,'Retract'; q_safe,'Safe Retract'; q_home,'Home'};

    for wi = 1:size(waypoints,1)
        if ~isAnimating, break; end
        setStatus(sprintf('Executing: %s (%d/%d)', waypoints{wi,2}, wi, size(waypoints,1)));
        animateToJoints(waypoints{wi,1});
        pause(0.05);
    end

    isAnimating = false;
    setStatus('Pick-place cycle complete');
end

function stopCB(~,~)
    isAnimating = false;
    setStatus('Stopped');
end

function clearTrailCB(~,~)
    trailXYZ = [];
    deleteValid(hTrailLine);
    hTrailLine = gobjects(0);
end

function animateToJoints(qTarget)
    nSteps = max(3, round(get(hSpeedSlider, 'Value')));
    q_start = q_current;
    for si = 1:nSteps
        if ~isAnimating && si > 1
            % 如果不在执行任务, 也允许单独animate (moveToXxx)
            % 只在executeCB中设置isAnimating
        end
        t = si / nSteps;
        q_interp = (1-t) * q_start + t * qTarget;
        q_current = q_interp;

        % 更新滑块
        for ji = 1:6
            set(hSliders(ji), 'Value', rad2deg(q_current(ji)));
            set(hSliderLabels(ji), 'String', sprintf('%+.1f°', rad2deg(q_current(ji))));
        end

        updateRobotDisplay();

        % 记录TCP轨迹
        T_all = FK_local(q_current);
        tcp = Tb * T_all{7};
        trailXYZ = [trailXYZ; tcp(1:3,4)'];

        % 更新轨迹线
        deleteValid(hTrailLine);
        if size(trailXYZ,1) > 1
            hTrailLine = plot3(ax3d, trailXYZ(:,1), trailXYZ(:,2), trailXYZ(:,3), ...
                '-', 'Color', [1 0.3 0 0.85], 'LineWidth', 2.5);
        end

        drawnow limitrate;
        pause(0.02);
    end
    q_current = qTarget;
    updateRobotDisplay();
end

function onClose(~,~)
    isAnimating = false;
    delete(fig);
end

%% ========================================================================
%%                       静态场景绘图
%% ========================================================================
function drawStaticScene()
    % 地面
    patch(ax3d, 'Vertices',[-1.5 -2 0;2 -2 0;2 3 0;-1.5 3 0],'Faces',[1 2 3 4],...
          'FaceColor',[.92 .92 .90],'EdgeColor','none','FaceAlpha',0.4);

    % 柜体
    c = defScene.cab;
    drawBox3D(ax3d, -c.wx/2, -c.dy/2, 0, c.wx, c.dy, c.hz, c.color);

    % 蓝框 (center X = 0)
    f = defScene.frame;
    fcx = 0;  % frame center X
    corners = [fcx-f.wx/2 frameCY-f.dy/2; fcx+f.wx/2 frameCY-f.dy/2;
               fcx+f.wx/2 frameCY+f.dy/2; fcx-f.wx/2 frameCY+f.dy/2];
    % 立柱
    for ii=1:4
        drawTube(ax3d,corners(ii,1),corners(ii,2),0,...
                 corners(ii,1),corners(ii,2),f.h,f.tubeR,f.color,CYL_N);
    end
    % 横梁 (3面, -Y开口)
    edges={[1,2],[2,3],[3,4],[4,1]};
    for hz_lev=[0.05 f.h/3 2*f.h/3 f.h-0.05]
        for ei=2:4
            i1=edges{ei}(1); i2=edges{ei}(2);
            drawTube(ax3d,corners(i1,1),corners(i1,2),hz_lev,...
                     corners(i2,1),corners(i2,2),hz_lev,f.tubeR*0.8,f.color,CYL_N);
        end
    end

    % 托盘
    px=fcx-defScene.pallet.wx/2; py=frameCY-defScene.pallet.dy/2;
    drawBox3D(ax3d, px, py, 0, defScene.pallet.wx, defScene.pallet.dy, defScene.pallet.hz, defScene.pallet.color);
    text(ax3d, fcx, frameCY, defScene.pallet.hz+0.03, ...
        sprintf('Pallet %dcm',round(defScene.pallet.hz*100)), ...
        'FontSize',11,'HorizontalAlignment','center','FontWeight','bold',...
        'FontName',CJK_FONT,'Color',[0.05 0.15 0.45]);

    % 传送带
    cv = defScene.conv;
    ly=cv.ly; wx=cv.wx; hz=cv.hz;
    x0=convCX-wx/2; y0=convCY-ly/2;
    % 侧板
    drawBox3D(ax3d,x0-0.015,y0,hz-0.06,0.015,ly,0.06,[0.4 0.4 0.42]);
    drawBox3D(ax3d,x0+wx,y0,hz-0.06,0.015,ly,0.06,[0.4 0.4 0.42]);
    % 支撑腿
    lw=0.035; yL=[y0+0.2 convCY y0+ly-0.2];
    for yi=1:3
        for s=[-1 1]
            lx=convCX+s*(wx/2-0.06);
            drawBox3D(ax3d,lx-lw/2,yL(yi)-lw/2,0,lw,lw,hz-0.01,[0.25 0.25 0.25]);
        end
    end
    % 滚筒
    sp=ly/(cv.nRollers+1);
    for ri=1:cv.nRollers
        ry=y0+ri*sp;
        [X,Y,Z]=cylinder(cv.rollerR,CYL_N); Z=Z*wx*0.9-wx*0.9/2;
        surf(ax3d,Z+convCX,zeros(size(X))+ry,X+hz+cv.rollerR,...
            'FaceColor',[0.55 0.55 0.55],'EdgeColor','none','FaceAlpha',0.6);
    end
    % 皮带
    bz=hz+cv.rollerR;
    bV=[x0+0.02 y0+0.04 bz;x0+wx-0.02 y0+0.04 bz;x0+wx-0.02 y0+ly-0.04 bz;x0+0.02 y0+ly-0.04 bz;
        x0+0.02 y0+0.04 bz+cv.beltH;x0+wx-0.02 y0+0.04 bz+cv.beltH;
        x0+wx-0.02 y0+ly-0.04 bz+cv.beltH;x0+0.02 y0+ly-0.04 bz+cv.beltH];
    patch(ax3d,'Vertices',bV,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',cv.color,'EdgeColor',[0.15 0.15 0.15],'FaceAlpha',0.92);
    text(ax3d, convCX, convCY, hz+cv.rollerR+cv.beltH+0.05, 'Conveyor', ...
        'FontSize',11,'HorizontalAlignment','center','FontWeight','bold',...
        'FontName',CJK_FONT,'Color',[0.2 0.2 0.3]);
end

%% ========================================================================
%%                       工具函数
%% ========================================================================
function setStatus(msg)
    set(hStatusText, 'String', ['  ' msg ' | HR_S50-2000 Interactive Simulator v1.0']);
    drawnow limitrate;
end

function deleteValid(h)
    if ~isempty(h)
        v = isvalid(h);
        if any(v), delete(h(v)); end
    end
end

function h = drawBox3D(ax, x,y,z,dx,dy,dz,col)
    v=[x y z;x+dx y z;x+dx y+dy z;x y+dy z;
       x y z+dz;x+dx y z+dz;x+dx y+dy z+dz;x y+dy z+dz];
    h = patch(ax,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],...
          'FaceColor',col,'EdgeColor',[0.3 0.3 0.3],'FaceAlpha',0.95,'LineWidth',1);
end

function [X,Y,Z] = capsuleMesh(radius, length, nSeg)
    % 创建胶囊体网格: 沿Z轴, 中心在原点, 从-L/2到+L/2
    L = length;
    nHemi = max(3, floor(nSeg/2));

    % 下半球
    tBot = linspace(-pi/2, 0, nHemi+1);
    rBot = radius * cos(tBot);
    zBot = radius * sin(tBot) - L/2;

    % 圆柱段
    rCyl = [radius; radius];
    zCyl = [-L/2; L/2];

    % 上半球
    tTop = linspace(0, pi/2, nHemi+1);
    rTop = radius * cos(tTop);
    zTop = radius * sin(tTop) + L/2;

    rAll = [rBot(:); rCyl(:); rTop(:)];
    zAll = [zBot(:); zCyl(:); zTop(:)];

    theta = linspace(0, 2*pi, nSeg+1);
    X = rAll * cos(theta);
    Y = rAll * sin(theta);
    Z = repmat(zAll, 1, nSeg+1);
end

function R = rotZtoDirection(d)
    % 旋转矩阵: 将 [0;0;1] 映射到方向 d
    d = d(:) / norm(d);
    z = [0;0;1];
    v = cross(z, d);
    s = norm(v);
    c = dot(z, d);
    if s < 1e-10
        if c > 0, R = eye(3);
        else, R = diag([-1, -1, 1]); end
        return;
    end
    vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    R = eye(3) + vx + vx*vx * (1-c)/(s*s);
end

function drawTube(ax, x1,y1,z1,x2,y2,z2,radius,color,cylN)
    [X,Y,Z]=cylinder(radius,cylN);
    v_dir=[x2-x1;y2-y1;z2-z1]; l=norm(v_dir);
    if l<0.001, return; end
    Z=Z*l; dd=[0;0;1]; td=v_dir/l; cp=cross(dd,td);
    if norm(cp)>1e-6
        ax_ang = acos(max(-1,min(1,dot(dd,td))));
        cpn = cp/norm(cp);
        c_a=cos(ax_ang); s_a=sin(ax_ang); t_a=1-c_a;
        RR=[t_a*cpn(1)^2+c_a t_a*cpn(1)*cpn(2)-s_a*cpn(3) t_a*cpn(1)*cpn(3)+s_a*cpn(2);
            t_a*cpn(1)*cpn(2)+s_a*cpn(3) t_a*cpn(2)^2+c_a t_a*cpn(2)*cpn(3)-s_a*cpn(1);
            t_a*cpn(1)*cpn(3)-s_a*cpn(2) t_a*cpn(2)*cpn(3)+s_a*cpn(1) t_a*cpn(3)^2+c_a];
    else
        RR=eye(3); if dot(dd,td)<0, RR(3,3)=-1; RR(1,1)=-1; end
    end
    for ii=1:numel(X)
        pt=RR*[X(ii);Y(ii);Z(ii)]; X(ii)=pt(1)+x1; Y(ii)=pt(2)+y1; Z(ii)=pt(3)+z1;
    end
    surf(ax,X,Y,Z,'FaceColor',color,'EdgeColor','none','FaceAlpha',0.85);
end

function fontName = selectFont()
    fontName = 'Noto Sans CJK SC';
    allF = listfonts();
    if any(strcmp(allF, fontName)), return; end
    cands = {'Noto Serif CJK SC','SimHei','WenQuanYi Micro Hei','Arial Unicode MS'};
    for ii = 1:length(cands)
        if any(strcmp(allF, cands{ii})), fontName = cands{ii}; return; end
    end
    fontName = get(0, 'DefaultAxesFontName');
end

end  % PalletizingSimApp main function
