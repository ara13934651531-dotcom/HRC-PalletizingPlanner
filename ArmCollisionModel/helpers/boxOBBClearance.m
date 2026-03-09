function [minDist, details] = boxOBBClearance(boxCenter, bx, yaw_rad, obstacles)
%BOXOBBCLEARANCE 计算旋转箱子OBB与环境障碍物的最小间隙
%   boxCenter  = [cx, cy, cz] - 箱子体积中心 (m)
%   bx         = struct with .lx, .wy, .hz - 箱子尺寸(m)
%   yaw_rad    = 绕Z轴旋转角度 (rad)
%   obstacles  = struct数组, 每个元素包含:
%     .type    = 'capsule' | 'ball' | 'plane'
%     .p1, .p2, .radius  (capsule: 两端点+半径, m)
%     .center, .radius    (ball: 中心+半径, m)
%     .point, .normal     (plane: 面上一点+法向量)
%     .name    = 障碍物名称 (可选)
%
%   返回:
%     minDist  = 最小间隙距离 (m), 负值=穿透
%     details  = struct with:
%       .cornerDists    = [nObstacles x 4] 每个障碍物对每个角的距离
%       .edgeDists      = [nObstacles x 4] 每个障碍物对每条边的距离
%       .worstCorner    = 最近角索引
%       .worstObstacle  = 最近障碍物索引
%       .worstObsName   = 最近障碍物名称
%
%   算法: 在XY平面上计算旋转矩形(箱子俯视投影)与障碍物的最小距离
%         Z方向间隙另外计算 (框架顶梁等)
%
%   @file   boxOBBClearance.m
%   @date   2026-02-25

    if nargin < 4 || isempty(obstacles)
        minDist = Inf;
        details = struct('cornerDists', [], 'edgeDists', [], ...
            'worstCorner', 0, 'worstObstacle', 0, 'worstObsName', '');
        return;
    end

    hx = bx.lx / 2;
    hy = bx.wy / 2;
    c = cos(yaw_rad);
    s = sin(yaw_rad);

    % 箱子4个角的XY坐标 (世界坐标系)
    localCorners = [-hx, -hy; hx, -hy; hx, hy; -hx, hy];
    corners2D = zeros(4, 2);
    for ci = 1:4
        corners2D(ci, 1) = c * localCorners(ci,1) - s * localCorners(ci,2) + boxCenter(1);
        corners2D(ci, 2) = s * localCorners(ci,1) + c * localCorners(ci,2) + boxCenter(2);
    end

    % 箱子4条边: edge(i) = corners(i) → corners(mod(i,4)+1)
    edges = zeros(4, 4);  % [x1, y1, x2, y2]
    for ei = 1:4
        ni = mod(ei, 4) + 1;
        edges(ei, :) = [corners2D(ei,:), corners2D(ni,:)];
    end

    nObs = length(obstacles);
    cornerDists = Inf(nObs, 4);
    edgeDists   = Inf(nObs, 4);
    minDist = Inf;
    worstCorner = 0;
    worstObstacle = 0;

    for oi = 1:nObs
        obs = obstacles(oi);
        switch lower(obs.type)
            case 'capsule'
                % 胶囊体 → XY平面投影为线段+半径
                seg_a = obs.p1(1:2);  % 端点1 XY
                seg_b = obs.p2(1:2);  % 端点2 XY
                r = obs.radius;

                % Z方向检查: 箱子是否在胶囊体Z范围内
                zOverlap = true;
                obsZmin = min(obs.p1(3), obs.p2(3)) - r;
                obsZmax = max(obs.p1(3), obs.p2(3)) + r;
                boxZmin = boxCenter(3) - bx.hz/2;
                boxZmax = boxCenter(3) + bx.hz/2;
                if boxZmax < obsZmin || boxZmin > obsZmax
                    zOverlap = false;
                end

                for ci = 1:4
                    d_xy = ptToSegDist2D(corners2D(ci,:), seg_a, seg_b) - r;
                    cornerDists(oi, ci) = d_xy;
                    if d_xy < minDist && zOverlap
                        minDist = d_xy;
                        worstCorner = ci;
                        worstObstacle = oi;
                    end
                end

                % 边对胶囊轴的距离
                for ei = 1:4
                    d_edge = segToSegDist2D(edges(ei,1:2), edges(ei,3:4), seg_a, seg_b) - r;
                    edgeDists(oi, ei) = d_edge;
                    if d_edge < minDist && zOverlap
                        minDist = d_edge;
                        worstCorner = -ei;  % 负值表示边
                        worstObstacle = oi;
                    end
                end

            case 'ball'
                % 球体 → XY平面投影为圆
                bc = obs.center(1:2);
                r = obs.radius;

                for ci = 1:4
                    d = norm(corners2D(ci,:) - bc) - r;
                    cornerDists(oi, ci) = d;
                    if d < minDist
                        minDist = d;
                        worstCorner = ci;
                        worstObstacle = oi;
                    end
                end

                for ei = 1:4
                    d_edge = ptToSegDist2D(bc, edges(ei,1:2), edges(ei,3:4)) - r;
                    edgeDists(oi, ei) = d_edge;  % 注意: 这里是点到线段距离
                    if d_edge < minDist
                        minDist = d_edge;
                        worstCorner = -ei;
                        worstObstacle = oi;
                    end
                end

            case 'plane'
                % 半平面约束 → 有符号距离
                n = obs.normal(1:2);
                p0 = obs.point(1:2);

                for ci = 1:4
                    d = dot(corners2D(ci,:) - p0, n);  % 有符号距离
                    cornerDists(oi, ci) = d;
                    if d < minDist
                        minDist = d;
                        worstCorner = ci;
                        worstObstacle = oi;
                    end
                end
        end
    end

    % 组装详情
    obsName = '';
    if worstObstacle > 0 && isfield(obstacles(worstObstacle), 'name')
        obsName = obstacles(worstObstacle).name;
    end

    details = struct();
    details.cornerDists = cornerDists;
    details.edgeDists = edgeDists;
    details.worstCorner = worstCorner;
    details.worstObstacle = worstObstacle;
    details.worstObsName = obsName;
    details.corners2D = corners2D;
end

%% ====== 局部辅助函数 ======

function d = ptToSegDist2D(p, a, b)
%PTTOSEGDIST2D 点到线段距离 (2D)
    ab = b - a;
    ap = p - a;
    lenSq = dot(ab, ab);
    if lenSq < 1e-12
        d = norm(ap);
        return;
    end
    t = dot(ap, ab) / lenSq;
    t = max(0, min(1, t));
    closest = a + t * ab;
    d = norm(p - closest);
end

function d = segToSegDist2D(a1, a2, b1, b2)
%SEGTOSEGDIST2D 线段到线段最小距离 (2D)
%   检查4种点-线段距离 + 交叉情况
    d1 = ptToSegDist2D(a1, b1, b2);
    d2 = ptToSegDist2D(a2, b1, b2);
    d3 = ptToSegDist2D(b1, a1, a2);
    d4 = ptToSegDist2D(b2, a1, a2);
    d = min([d1, d2, d3, d4]);

    % 检查是否相交
    if segmentsIntersect2D(a1, a2, b1, b2)
        d = 0;
    end
end

function intersect = segmentsIntersect2D(a1, a2, b1, b2)
%SEGMENTSINTERSECT2D 检查两线段是否相交 (2D)
    d1 = cross2D(b2 - b1, a1 - b1);
    d2 = cross2D(b2 - b1, a2 - b1);
    d3 = cross2D(a2 - a1, b1 - a1);
    d4 = cross2D(a2 - a1, b2 - a1);
    intersect = (d1 * d2 < 0) && (d3 * d4 < 0);
end

function v = cross2D(a, b)
    v = a(1) * b(2) - a(2) * b(1);
end
