%% test_screenshot_v2.m - 自动运行PalletizingSimApp并截屏分析
% 截屏: 1-初始界面, 2-添加箱子后, 3-执行动画后
% 输出到 ArmCollisionModel/pic/sim_test/

function test_screenshot_v2()

outDir = fullfile(fileparts(mfilename('fullpath')), 'pic', 'sim_test');
if ~exist(outDir, 'dir'), mkdir(outDir); end

fprintf('=== PalletizingSimApp v2.0 Screenshot Test ===\n');

%% 1. 启动应用
fprintf('[1/6] Launching app...\n');
PalletizingSimApp();
drawnow;
pause(1.5);

% 截屏1: 初始界面
fig = gcf;
saveas(fig, fullfile(outDir, '01_initial.png'));
fprintf('  -> Saved 01_initial.png\n');

%% 2. 添加第一个箱子 (模拟点击Add按钮)
fprintf('[2/6] Adding Box1...\n');
% 找到Add按钮并模拟点击
addBtn = findobj(fig, 'Style', 'pushbutton', 'String', 'Add');
if ~isempty(addBtn)
    cb = get(addBtn(1), 'Callback');
    cb(addBtn(1), []);
end
drawnow; pause(0.5);
saveas(fig, fullfile(outDir, '02_box1_added.png'));
fprintf('  -> Saved 02_box1_added.png\n');

%% 3. 添加第二个箱子
fprintf('[3/6] Adding Box2...\n');
if ~isempty(addBtn)
    cb = get(addBtn(1), 'Callback');
    cb(addBtn(1), []);
end
drawnow; pause(0.5);
saveas(fig, fullfile(outDir, '03_box2_added.png'));
fprintf('  -> Saved 03_box2_added.png\n');

%% 4. 添加第三个箱子
fprintf('[4/6] Adding Box3...\n');
if ~isempty(addBtn)
    cb = get(addBtn(1), 'Callback');
    cb(addBtn(1), []);
end
drawnow; pause(0.5);
saveas(fig, fullfile(outDir, '04_box3_added.png'));
fprintf('  -> Saved 04_box3_added.png\n');

%% 5. 执行Auto Pick & Place
fprintf('[5/6] Executing Auto Pick & Place...\n');
autoBtn = findobj(fig, 'Style', 'pushbutton', 'String', 'Auto Pick & Place');
if ~isempty(autoBtn)
    cb = get(autoBtn(1), 'Callback');
    cb(autoBtn(1), []);
end
drawnow; pause(1);
saveas(fig, fullfile(outDir, '05_after_execute.png'));
fprintf('  -> Saved 05_after_execute.png\n');

%% 6. 不同视角截屏
fprintf('[6/6] Capturing different views...\n');
ax = findobj(fig, 'Type', 'axes');
if ~isempty(ax)
    % 正面视角
    view(ax(end), 0, 0);
    drawnow; pause(0.3);
    saveas(fig, fullfile(outDir, '06_front_view.png'));
    fprintf('  -> Saved 06_front_view.png\n');
    
    % 俯视图
    view(ax(end), 0, 90);
    drawnow; pause(0.3);
    saveas(fig, fullfile(outDir, '07_top_view.png'));
    fprintf('  -> Saved 07_top_view.png\n');
    
    % 侧面视角
    view(ax(end), 90, 15);
    drawnow; pause(0.3);
    saveas(fig, fullfile(outDir, '08_side_view.png'));
    fprintf('  -> Saved 08_side_view.png\n');
    
    % 恢复默认视角
    view(ax(end), 135, 25);
    drawnow; pause(0.3);
    saveas(fig, fullfile(outDir, '09_default_view.png'));
    fprintf('  -> Saved 09_default_view.png\n');
end

fprintf('\n=== All screenshots saved to %s ===\n', outDir);
fprintf('=== Test complete ===\n');

% 关闭
pause(1);
close(fig);

end
