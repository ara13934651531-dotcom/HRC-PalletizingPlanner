%% test_helpers_unit.m — 辅助函数单元测试
% 测试所有活跃的MATLAB辅助函数正确性
% Usage: cd ArmCollisionModel && run('test_helpers_unit.m')
% Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.

clear; clc;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'helpers'));
cd(fileparts(scriptDir));  % cd to project root for data/ access

nPass = 0; nFail = 0; nTotal = 0;
results = {};

%% ═══════════════════ 数学工具测试 ═══════════════════════════
fprintf('\n══════ 数学工具测试 ══════\n\n');

% Test 1: axang2r_local
nTotal = nTotal + 1;
try
    R = axang2r_local([0 0 1 pi/2]);
    assert(abs(R(1,2)+1) < 1e-10 && abs(R(2,1)-1) < 1e-10, 'rotation matrix incorrect');
    R2 = axang2r_local([1 0 0 0]); % zero rotation
    assert(norm(R2-eye(3)) < 1e-10, 'zero rotation should be identity');
    fprintf('  [PASS] axang2r_local: 轴角→旋转矩阵\n'); nPass = nPass + 1;
    results{end+1} = {'axang2r_local', 'PASS', ''};
catch e
    fprintf('  [FAIL] axang2r_local: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'axang2r_local', 'FAIL', e.message};
end

% Test 2: makeTrans
nTotal = nTotal + 1;
try
    T = makeTrans([1 2 3]);
    assert(all(T(1:3,4)==[1;2;3]) && all(all(T(1:3,1:3)==eye(3))), 'translation matrix wrong');
    fprintf('  [PASS] makeTrans: 4x4平移矩阵\n'); nPass = nPass + 1;
    results{end+1} = {'makeTrans', 'PASS', ''};
catch e
    fprintf('  [FAIL] makeTrans: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'makeTrans', 'FAIL', e.message};
end

% Test 3: makeRotX/Y/Z
nTotal = nTotal + 1;
try
    Rx = makeRotX(pi/2); assert(abs(Rx(2,3)+1)<1e-10, 'Rx wrong');
    Ry = makeRotY(pi/2); assert(abs(Ry(1,3)-1)<1e-10, 'Ry wrong');
    Rz = makeRotZ(pi/2); assert(abs(Rz(1,2)+1)<1e-10, 'Rz wrong');
    % Verify orthogonality
    assert(abs(det(Rx)-1)<1e-10, 'Rx not orthogonal');
    assert(abs(det(Ry)-1)<1e-10, 'Ry not orthogonal');
    assert(abs(det(Rz)-1)<1e-10, 'Rz not orthogonal');
    fprintf('  [PASS] makeRotX/Y/Z: 旋转矩阵 + 正交性验证\n'); nPass = nPass + 1;
    results{end+1} = {'makeRotX/Y/Z', 'PASS', ''};
catch e
    fprintf('  [FAIL] makeRotX/Y/Z: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'makeRotX/Y/Z', 'FAIL', e.message};
end

% Test 4: makeRotRPY
nTotal = nTotal + 1;
try
    R4 = makeRotRPY([pi/4 0 0]);
    assert(norm(R4-makeRotX(pi/4))<1e-10, 'RPY with only roll should equal RotX');
    R5 = makeRotRPY([0 pi/3 0]);
    assert(norm(R5-makeRotY(pi/3))<1e-10, 'RPY with only pitch should equal RotY');
    R6 = makeRotRPY([0 0 pi/6]);
    assert(norm(R6-makeRotZ(pi/6))<1e-10, 'RPY with only yaw should equal RotZ');
    fprintf('  [PASS] makeRotRPY: RPY→旋转矩阵 (ZYX顺序)\n'); nPass = nPass + 1;
    results{end+1} = {'makeRotRPY', 'PASS', ''};
catch e
    fprintf('  [FAIL] makeRotRPY: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'makeRotRPY', 'FAIL', e.message};
end

%% ═══════════════════ FK 运动学测试 ═══════════════════════════
fprintf('\n══════ FK运动学测试 ══════\n\n');

% Test 5: fk2Skeleton
nTotal = nTotal + 1;
try
    j = fk2Skeleton([0 0 0 0 0 0]);
    assert(size(j,1)==5 && size(j,2)==3, 'fk2Skeleton must return 5x3');
    assert(all(j(1,:)==0), 'Base joint should be at origin');
    assert(j(2,3) > 0, 'Shoulder should be above base');
    % HOME config (compressed model, conventions differ from FK2)
    jh = fk2Skeleton([0 -90 0 0 90 0]);
    assert(size(jh,1)==5 && size(jh,2)==3, 'HOME must also return 5x3');
    fprintf('  [PASS] fk2Skeleton: ZERO_tcp=[%.3f,%.3f,%.3f]m HOME_tcp=[%.3f,%.3f,%.3f]m\n', ...
        j(5,:), jh(5,:));
    nPass = nPass + 1;
    results{end+1} = {'fk2Skeleton', 'PASS', ''};
catch e
    fprintf('  [FAIL] fk2Skeleton: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'fk2Skeleton', 'FAIL', e.message};
end

% Test 6: urdfFK (takes Nx6 matrix, not struct array)
nTotal = nTotal + 1;
try
    JOINTS_mat = [
        0,       0, 0.2833,        0,      0,  pi/2;
       -0.3345,  0, 0,          pi/2,      0, -pi/2;
       -0.9,     0, -0.239,        0,      0,  pi;
        0.9415,  0, 0,              0,      0,  0;
        0,       0, 0.1585,    -pi/2,      0,  0;
        0,       0, 0.1585,     pi/2,      0,  0;
    ];
    T_all = urdfFK(JOINTS_mat, zeros(1,6));
    assert(length(T_all)==8, 'urdfFK should return 8 transforms');
    assert(all(size(T_all{1})==[4 4]), 'Each transform should be 4x4');
    tcp_pos = T_all{8}(1:3,4);
    assert(norm(tcp_pos) > 0.5, 'TCP should be far from origin');
    fprintf('  [PASS] urdfFK: ZERO TCP=[%.3f,%.3f,%.3f]m (%d transforms)\n', tcp_pos, length(T_all));
    nPass = nPass + 1;
    results{end+1} = {'urdfFK', 'PASS', ''};
catch e
    fprintf('  [FAIL] urdfFK: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'urdfFK', 'FAIL', e.message};
end

%% ═══════════════════ 通用工具测试 ═══════════════════════════
fprintf('\n══════ 通用工具测试 ══════\n\n');

% Test 7: getField
nTotal = nTotal + 1;
try
    s.a=1; s.b='hello';
    assert(getField(s,'a',0)==1, 'existing field');
    assert(getField(s,'c',99)==99, 'missing field default');
    assert(strcmp(getField(s,'b',''), 'hello'), 'string field');
    fprintf('  [PASS] getField: 安全字段访问 (存在/缺省/字符串)\n'); nPass = nPass + 1;
    results{end+1} = {'getField', 'PASS', ''};
catch e
    fprintf('  [FAIL] getField: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'getField', 'FAIL', e.message};
end

% Test 8: ifelse
nTotal = nTotal + 1;
try
    assert(ifelse(true, 10, 20)==10, 'true case');
    assert(ifelse(false, 10, 20)==20, 'false case');
    r = ifelse(1>0, 'yes', 'no'); assert(strcmp(r,'yes'), 'string result');
    fprintf('  [PASS] ifelse: 条件选择 (true/false/string)\n'); nPass = nPass + 1;
    results{end+1} = {'ifelse', 'PASS', ''};
catch e
    fprintf('  [FAIL] ifelse: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'ifelse', 'FAIL', e.message};
end

%% ═══════════════════ 数据I/O测试 ═══════════════════════════
fprintf('\n══════ 数据I/O测试 ══════\n\n');

% Test 9: loadNumericData
nTotal = nTotal + 1;
try
    D = loadNumericData('data/so_palletizing_trajectory.txt');
    assert(size(D,2)==19, sprintf('Expected 19 cols, got %d', size(D,2)));
    assert(size(D,1)>10, 'Should have many rows');
    assert(~any(isnan(D(:))), 'No NaN values expected');
    fprintf('  [PASS] loadNumericData: %dx%d matrix loaded, no NaN\n', size(D,1), size(D,2));
    nPass = nPass + 1;
    results{end+1} = {'loadNumericData', 'PASS', ''};
catch e
    fprintf('  [FAIL] loadNumericData: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'loadNumericData', 'FAIL', e.message};
end

% Test 10: readSummaryFile
nTotal = nTotal + 1;
try
    S = readSummaryFile('data/so_palletizing_summary.txt');
    assert(isfield(S,'version'), 'missing version field');
    assert(isfield(S,'positions'), 'missing positions field');
    fprintf('  [PASS] readSummaryFile: version=%s, positions=%s\n', S.version, S.positions);
    nPass = nPass + 1;
    results{end+1} = {'readSummaryFile', 'PASS', ''};
catch e
    fprintf('  [FAIL] readSummaryFile: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'readSummaryFile', 'FAIL', e.message};
end

%% ═══════════════════ 碰撞分析测试 (v18.0) ═══════════════════
fprintf('\n══════ 碰撞分析测试 (v18.0) ══════\n\n');

% Test 11: boxOBBClearance (basic geometry)
% Signature: boxOBBClearance(boxCenter, bx_struct, yaw_rad, obstacles_struct_array)
nTotal = nTotal + 1;
try
    bx_test = struct('lx',0.35, 'wy',0.28, 'hz',0.25);
    obs_far = struct('type','capsule','p1',[5,0,0],'p2',[5,0,1],'radius',0.1,'name','far');
    [clr, ~] = boxOBBClearance([0,0,0], bx_test, 0, obs_far);
    assert(clr > 3.0, sprintf('Clearance should be >3m, got %.2f', clr));
    % Box near obstacle
    obs_near = struct('type','capsule','p1',[0.7,0,0],'p2',[0.7,0,1],'radius',0.1,'name','near');
    [clr2, ~] = boxOBBClearance([0.5,0,0], bx_test, 0, obs_near);
    assert(clr2 < clr, 'Closer box should have less clearance');
    fprintf('  [PASS] boxOBBClearance: far=%.2fm, near=%.2fm (monotonic)\n', clr, clr2);
    nPass = nPass + 1;
    results{end+1} = {'boxOBBClearance', 'PASS', ''};
catch e
    fprintf('  [FAIL] boxOBBClearance: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'boxOBBClearance', 'FAIL', e.message};
end

% Test 12: optimizeBoxRotation
% Signature: optimizeBoxRotation(boxCenter, bx_struct, obstacles_struct_array)
nTotal = nTotal + 1;
try
    bx_test2 = struct('lx',0.35, 'wy',0.28, 'hz',0.25);
    obs_opt = struct('type','capsule','p1',[0.6,0,0],'p2',[0.6,0,1],'radius',0.05,'name','pillar');
    [bestYaw, bestClr, ~] = optimizeBoxRotation([0.35,0,0.125], bx_test2, obs_opt);
    assert(isscalar(bestYaw) && isscalar(bestClr), 'Should return scalars');
    assert(abs(bestYaw) <= pi, 'Yaw should be in [-pi, pi]');
    fprintf('  [PASS] optimizeBoxRotation: bestYaw=%.1f deg, clearance=%.3fm\n', ...
        rad2deg(bestYaw), bestClr);
    nPass = nPass + 1;
    results{end+1} = {'optimizeBoxRotation', 'PASS', ''};
catch e
    fprintf('  [FAIL] optimizeBoxRotation: %s\n', e.message); nFail = nFail + 1;
    results{end+1} = {'optimizeBoxRotation', 'FAIL', e.message};
end

%% ═══════════════════ 结果汇总 ═══════════════════════════════
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║        辅助函数单元测试报告 v18.0                ║\n');
fprintf('╠══════════════════════════════════════════════════╣\n');
fprintf('║  总测试: %2d  通过: %2d  失败: %2d               ║\n', nTotal, nPass, nFail);
fprintf('║  通过率: %5.1f%%                                 ║\n', 100*nPass/nTotal);
fprintf('╚══════════════════════════════════════════════════╝\n');

% Save results for report
save(fullfile(scriptDir, 'data', 'test_helpers_results.mat'), 'results', 'nPass', 'nFail', 'nTotal');

if nFail > 0
    fprintf('\n⚠ %d tests FAILED!\n', nFail);
    for ri = 1:length(results)
        if strcmp(results{ri}{2}, 'FAIL')
            fprintf('  - %s: %s\n', results{ri}{1}, results{ri}{3});
        end
    end
    exit(1);
else
    fprintf('\n✅ All %d tests PASSED\n', nTotal);
    exit(0);
end
