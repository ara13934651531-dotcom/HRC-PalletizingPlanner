function [bestYaw, bestClearance, profile] = optimizeBoxRotation(boxCenter, bx, obstacles, yawRange_rad, nSteps)
%OPTIMIZEBOXROTATION 搜索最优TCP Z轴旋转角度以最大化箱子间隙
%   boxCenter    = [cx, cy, cz] - 箱子体积中心 (m)
%   bx           = struct with .lx, .wy, .hz - 箱子尺寸(m)
%   obstacles    = struct数组 (与boxOBBClearance兼容)
%   yawRange_rad = [lo, hi] 搜索范围 (rad), 默认 [-pi/2, pi/2]
%   nSteps       = 搜索分辨率, 默认 72 (每5度一个采样)
%
%   返回:
%     bestYaw       = 最优旋转角度 (rad)
%     bestClearance = 最优间隙距离 (m)
%     profile       = struct with:
%       .yaws        = [1 x nSteps] 采样角度
%       .clearances  = [1 x nSteps] 对应间隙
%       .worstObs    = [1 x nSteps] 对应最近障碍物索引
%
%   算法: 均匀扫描 yaw 范围, 对每个角度调用 boxOBBClearance,
%         选取间隙最大的角度。精度由 nSteps 控制。
%
%   适用场景:
%     - 搬运段(seg 3-5): 箱子跟随TCP移动, 旋转箱子避开框架立柱/墙面
%     - 进入框架入口: 箱子长轴(350mm)对齐通行方向, 减少横向投影
%     - TCP水平约束下: 仅J6影响箱子水平旋转, 不影响TCP高度
%
%   @file   optimizeBoxRotation.m
%   @date   2026-02-25

    if nargin < 4 || isempty(yawRange_rad)
        yawRange_rad = [-pi/2, pi/2];
    end
    if nargin < 5 || isempty(nSteps)
        nSteps = 72;
    end

    yaws = linspace(yawRange_rad(1), yawRange_rad(2), nSteps);
    clearances = zeros(1, nSteps);
    worstObs = zeros(1, nSteps);

    for i = 1:nSteps
        [clearances(i), details] = boxOBBClearance(boxCenter, bx, yaws(i), obstacles);
        worstObs(i) = details.worstObstacle;
    end

    [bestClearance, bestIdx] = max(clearances);
    bestYaw = yaws(bestIdx);

    % 局部细化 (可选: 在最优点附近±1步细化)
    if nSteps > 3
        step = (yawRange_rad(2) - yawRange_rad(1)) / (nSteps - 1);
        refineRange = [bestYaw - step, bestYaw + step];
        refineRange = max(refineRange, yawRange_rad(1));
        refineRange = min(refineRange, yawRange_rad(2));
        nRefine = 20;
        yawsRefine = linspace(refineRange(1), refineRange(2), nRefine);
        for i = 1:nRefine
            [cR, ~] = boxOBBClearance(boxCenter, bx, yawsRefine(i), obstacles);
            if cR > bestClearance
                bestClearance = cR;
                bestYaw = yawsRefine(i);
            end
        end
    end

    profile.yaws = yaws;
    profile.clearances = clearances;
    profile.worstObs = worstObs;
end
