function [dists, report] = boxArmCollDist(q_deg, tcpPos_mm, tcpABC_deg, boxDims, SO_HEADER, SO_ALIAS)
% boxArmCollDist  独立箱子-机械臂碰撞距离分析 (匹配C++ BoxCollisionChecker)
%
%   [dists, report] = boxArmCollDist(q_deg, tcpPos_mm, tcpABC_deg, boxDims)
%   [dists, report] = boxArmCollDist(q_deg, tcpPos_mm, tcpABC_deg, boxDims, SO_HEADER, SO_ALIAS)
%
% 输入:
%   q_deg       - 6×1 关节角 (deg)
%   tcpPos_mm   - 3×1 TCP世界坐标 (mm) — 从轨迹文件 cols 17-19
%   tcpABC_deg  - 3×1 TCP朝向 (A,B,C deg) — 从FK2获取
%   boxDims     - struct: .LX, .WY, .HZ (mm), .offsetX/Y/Z (默认0), .margin (默认20)
%   SO_HEADER   - SO库头文件路径 (可选, 默认 's50_collision_matlab.h')
%   SO_ALIAS    - SO库别名 (可选, 默认 'libHRCInterface')
%
% 输出:
%   dists       - 5×1 各碰撞体最小表面距离 (mm): [Base, LowArm, Elbow, UpArm, Wrist]
%   report      - struct:
%       .collision       - 是否碰撞 (仅Base+LowerArm判定)
%       .criticalMin     - 安全关键最小距离 (Base+LowArm, mm)
%       .criticalName    - 安全关键最近碰撞体名称
%       .overallMin      - 所有碰撞体最小距离 (mm, 诊断用)
%       .overallName     - 所有碰撞体中最近的名称
%       .colliderNames   - 5×1 cell: 碰撞体名称
%
% 碰撞安全策略 (v6.3):
%   ★ 硬约束 (规划器拒绝): Base(idx=1) + LowerArm(idx=2)
%   诊断 (不拒绝): Elbow(3) / UpArm(4) / Wrist(5) — 结构性相邻TCP
%
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

if nargin < 5, SO_ALIAS = 'libHRCInterface'; end
if nargin < 6, SO_HEADER = 's50_collision_matlab.h'; end %#ok<NASGU>

% ── 默认值 ────────────────────────────────────────────────
if ~isfield(boxDims, 'offsetX'), boxDims.offsetX = 0; end
if ~isfield(boxDims, 'offsetY'), boxDims.offsetY = 0; end
if ~isfield(boxDims, 'offsetZ'), boxDims.offsetZ = 0; end
if ~isfield(boxDims, 'margin'),  boxDims.margin  = 20; end

colliderNames = {'Base','LowArm','Elbow','UpArm','Wrist'};

% ── 1. 更新关节状态 ──────────────────────────────────────
q_copy = double(q_deg(:))';
vel = zeros(1,6); acc = zeros(1,6);
calllib(SO_ALIAS, 'updateACAreaConstrainPackageInterface', q_copy, vel, acc);

% ── 2. 获取碰撞体世界坐标 ────────────────────────────────
idx  = zeros(1,7,'int32');
typ  = zeros(1,7,'int32');
dat  = zeros(7,9);
rad  = zeros(1,7);
[idx, typ, dat, rad] = calllib(SO_ALIAS, 'getUIInfoMationInterface', idx, typ, dat, rad);

% ── 3. 箱子26采样点 (匹配C++ generateBoxSamplePoints) ────
R = eulerZYX_local(tcpABC_deg(1), tcpABC_deg(2), tcpABC_deg(3));
hx = boxDims.LX / 2;
hy = boxDims.WY / 2;
hz = boxDims.HZ;
pts = zeros(26, 3);
ci = 0;
corners = zeros(8,3);
for sx = [-1, 1]
    for sy = [-1, 1]
        for sz = [0, 1]
            ci = ci + 1;
            % TCP +Z = 世界下方 (箱子在TCP下方)
            local = [sx*hx + boxDims.offsetX;
                     sy*hy + boxDims.offsetY;
                     sz*hz + boxDims.offsetZ];
            corners(ci,:) = tcpPos_mm(:)' + (R * local)';
            pts(ci,:) = corners(ci,:);
        end
    end
end
% 12条边中点
edges = [1,2; 3,4; 5,6; 7,8; 1,3; 2,4; 5,7; 6,8; 1,5; 2,6; 3,7; 4,8];
for e = 1:12
    ci = ci + 1;
    pts(ci,:) = 0.5 * (corners(edges(e,1),:) + corners(edges(e,2),:));
end
% 6个面中心
faces = [5,6,7,8; 1,2,3,4; 3,4,7,8; 1,2,5,6; 1,3,5,7; 2,4,6,8];
for f = 1:6
    ci = ci + 1;
    pts(ci,:) = 0.25 * sum(corners(faces(f,:),:), 1);
end

% ── 4. 计算各碰撞体距离 ──────────────────────────────────
dists = inf(5,1);
for c = 1:7
    ci_idx = idx(c);  % 碰撞体索引 (1=Base, ..., 5=Wrist, 6/7=Tool)
    if ci_idx < 1 || ci_idx > 5 || rad(c) < 1, continue; end
    
    p1 = dat(c, 1:3);
    p2 = dat(c, 4:6);
    r  = rad(c);
    
    mn = inf;
    for pi = 1:26
        if typ(c) == 2  % 胶囊
            d = ptSegDist_local(pts(pi,:), p1, p2) - r;
        else  % 球
            d = norm(pts(pi,:) - p1) - r;
        end
        mn = min(mn, d);
    end
    dists(ci_idx) = mn;
end

% ── 5. 构建报告 ──────────────────────────────────────────
report.colliderNames = colliderNames;

% 安全关键: 仅 Base(1) + LowerArm(2)
critDists = dists(1:2);
[report.criticalMin, ci_crit] = min(critDists);
report.criticalName = colliderNames{ci_crit};
report.collision = report.criticalMin < boxDims.margin;

% 全局最小 (诊断用)
[report.overallMin, ci_all] = min(dists);
report.overallName = colliderNames{ci_all};

end

% ── 辅助: Euler ZYX旋转矩阵 ──────────────────────────────
function R = eulerZYX_local(A_deg, B_deg, C_deg)
    a = deg2rad(A_deg); b = deg2rad(B_deg); c = deg2rad(C_deg);
    ca=cos(a); sa=sin(a); cb=cos(b); sb=sin(b); cc=cos(c); sc=sin(c);
    R = [ca*cb, ca*sb*sc - sa*cc, ca*sb*cc + sa*sc;
         sa*cb, sa*sb*sc + ca*cc, sa*sb*cc - ca*sc;
         -sb,   cb*sc,            cb*cc           ];
end

% ── 辅助: 点到线段距离 ───────────────────────────────────
function d = ptSegDist_local(p, a, b)
    ab = b - a;
    ap = p - a;
    ab2 = dot(ab, ab);
    if ab2 < 1e-12
        d = norm(ap);
        return;
    end
    t = max(0, min(1, dot(ap, ab) / ab2));
    d = norm(p - (a + t * ab));
end
