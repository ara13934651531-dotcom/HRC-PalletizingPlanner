% Screenshot test for PalletizingSimApp v2.1 — capture before/after fix comparison
fprintf('=== PalletizingSimApp v2.1 Screenshot Capture ===\n');
try
    set(0,'DefaultFigureVisible','off');
    outDir = fullfile(pwd, 'pic', 'sim_test_v21');
    if ~exist(outDir,'dir'), mkdir(outDir); end

    % Launch app
    PalletizingSimApp();
    pause(1);
    fig = findall(0,'Type','figure');
    fig = fig(1);
    set(fig, 'Position', [40 40 1750 950]);
    drawnow;

    % 1. Initial state — no markers
    ax = findall(fig,'Type','axes'); ax = ax(1);
    view(ax, 135, 25);
    drawnow; pause(0.5);
    print(fig, fullfile(outDir, '01_initial_clean.png'), '-dpng', '-r150');
    fprintf('Saved 01_initial_clean.png\n');

    % 2. Add 3 boxes
    addBtn = findall(fig,'Style','pushbutton','String','Add');
    cb = get(addBtn(1),'Callback');
    for bi = 1:3
        nameEdit = findall(fig,'Style','edit');
        for ne = 1:length(nameEdit)
            s = get(nameEdit(ne),'String');
            if contains(s,'Box'), set(nameEdit(ne),'String',sprintf('Box%d',bi)); break; end
        end
        cb(addBtn(1),[]);
        pause(0.3);
    end
    drawnow; pause(0.5);
    print(fig, fullfile(outDir, '02_three_boxes.png'), '-dpng', '-r150');
    fprintf('Saved 02_three_boxes.png\n');

    % 3. Close-up robot (check opacity)
    view(ax, 90, 10);
    drawnow; pause(0.3);
    print(fig, fullfile(outDir, '03_robot_opacity.png'), '-dpng', '-r150');
    fprintf('Saved 03_robot_opacity.png\n');

    % 4. Execute Auto Pick & Place
    view(ax, 135, 25);
    autoBtn = findall(fig,'Style','pushbutton','String','Auto Pick & Place');
    cb2 = get(autoBtn(1),'Callback');
    cb2(autoBtn(1),[]);
    pause(5);
    drawnow;
    print(fig, fullfile(outDir, '04_after_cycle1.png'), '-dpng', '-r150');
    fprintf('Saved 04_after_cycle1.png\n');

    % 5. Top view
    view(ax, 0, 90);
    drawnow; pause(0.3);
    print(fig, fullfile(outDir, '05_top_after_cycle1.png'), '-dpng', '-r150');
    fprintf('Saved 05_top_after_cycle1.png\n');

    % 6. Add another box and run again
    view(ax, 135, 25);
    nameEdit = findall(fig,'Style','edit');
    for ne = 1:length(nameEdit)
        s = get(nameEdit(ne),'String');
        if contains(s,'Box'), set(nameEdit(ne),'String','Box4'); break; end
    end
    addCb = get(addBtn(1),'Callback');
    addCb(addBtn(1),[]);
    pause(0.3);
    cb2(autoBtn(1),[]);
    pause(5);
    drawnow;
    print(fig, fullfile(outDir, '06_after_cycle2.png'), '-dpng', '-r150');
    fprintf('Saved 06_after_cycle2.png\n');

    % 7. Pallet view
    view(ax, 180, 20);
    drawnow; pause(0.3);
    print(fig, fullfile(outDir, '07_pallet_view.png'), '-dpng', '-r150');
    fprintf('Saved 07_pallet_view.png\n');

    % 8. Default view final
    view(ax, 135, 25);
    drawnow; pause(0.3);
    print(fig, fullfile(outDir, '08_final_default.png'), '-dpng', '-r150');
    fprintf('Saved 08_final_default.png\n');

    close(fig);
    fprintf('\n=== Screenshots saved to %s ===\n', outDir);

catch ME
    fprintf('ERROR: %s\n  at %s line %d\n', ME.message, ME.stack(1).name, ME.stack(1).line);
    for si = 1:min(5,length(ME.stack))
        fprintf('  > %s line %d\n', ME.stack(si).name, ME.stack(si).line);
    end
end
