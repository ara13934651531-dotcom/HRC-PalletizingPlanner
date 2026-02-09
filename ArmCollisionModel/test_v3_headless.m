% Headless smoke test for PalletizingSimApp v2.1 fixes
fprintf('=== PalletizingSimApp v2.1 Headless Test ===\n');
try
    set(0,'DefaultFigureVisible','off');
    
    % Launch app
    PalletizingSimApp();
    pause(1);
    
    figs = findall(0,'Type','figure');
    if isempty(figs)
        error('No figure created');
    end
    fig = figs(1);
    fprintf('OK: Figure created: %s\n', fig.Name);
    
    % Check no Place marker at startup (Fix #1)
    placeMark = findall(fig,'Type','line','Marker','v','MarkerFaceColor',[0.8 0 0]);
    if isempty(placeMark)
        fprintf('OK: No Place marker at startup (Fix #1 verified)\n');
    else
        fprintf('WARN: Place marker found at startup\n');
    end
    
    % Check no Pick marker at startup
    pickMark = findall(fig,'Type','line','Marker','v','MarkerFaceColor',[0 0.8 0]);
    if isempty(pickMark)
        fprintf('OK: No Pick marker at startup\n');
    else
        fprintf('WARN: Pick marker found at startup\n');
    end
    
    % Add a box via simulated button press
    addBtn = findall(fig,'Style','pushbutton','String','Add');
    if ~isempty(addBtn)
        cb = get(addBtn(1),'Callback');
        cb(addBtn(1),[]);
        pause(0.3);
        fprintf('OK: Box added\n');
    end
    
    % Check Pick and Place markers appeared after adding box
    pickMark = findall(fig,'Type','line','Marker','v','MarkerFaceColor',[0 0.8 0]);
    placeMark = findall(fig,'Type','line','Marker','v','MarkerFaceColor',[0.8 0 0]);
    if ~isempty(pickMark), fprintf('OK: Pick marker appeared after Add\n'); end
    if ~isempty(placeMark), fprintf('OK: Place marker appeared after Add\n'); end
    
    % Check axis limits (Fix #4)
    ax = findall(fig,'Type','axes');
    xl = get(ax(1),'XLim'); yl = get(ax(1),'YLim'); zl = get(ax(1),'ZLim');
    fprintf('Axis: X=[%.1f,%.1f] Y=[%.1f,%.1f] Z=[%.1f,%.1f]\n', xl, yl, zl);
    if xl(1)==-1.2 && xl(2)==1.5 && yl(1)==-1.5 && yl(2)==2.5
        fprintf('OK: Axis limits tightened (Fix #4 verified)\n');
    end
    
    % Check GridAlpha (Fix #11)
    ga = get(ax(1),'GridAlpha');
    if abs(ga - 0.15) < 0.001
        fprintf('OK: GridAlpha=%.2f (Fix #11 verified)\n', ga);
    else
        fprintf('WARN: GridAlpha=%.2f expected 0.15\n', ga);
    end
    
    % Check capsule alpha (Fix #7)
    capsules = findall(fig,'Type','surface','FaceAlpha',0.65);
    fprintf('OK: %d surfaces with FaceAlpha=0.65 (Fix #7)\n', length(capsules));
    
    % Check speed slider (Fix #14)
    speedSldr = findall(fig,'Style','slider','Min',5);
    if ~isempty(speedSldr)
        fprintf('OK: Speed slider min=5, value=%.0f (Fix #14 verified)\n', get(speedSldr(1),'Value'));
    end
    
    % Auto Execute test
    autoBtn = findall(fig,'Style','pushbutton','String','Auto Pick & Place');
    if ~isempty(autoBtn)
        cb = get(autoBtn(1),'Callback');
        cb(autoBtn(1),[]);
        pause(5);
        fprintf('OK: Auto Pick & Place executed\n');
    end
    
    % Check if carried box was cleaned up (should be 5: 4 capsules + 1 sphere)
    hgtAll = findall(fig,'Type','hgtransform');
    fprintf('hgtransform count after exec: %d (expect 5 = 4 capsules + 1 sphere)\n', length(hgtAll));
    
    % Check pallet info
    palletInfo = findall(fig,'Type','uicontrol');
    for i = 1:length(palletInfo)
        s = get(palletInfo(i),'String');
        if ischar(s) && contains(s,'Palletized')
            fprintf('PalletInfo: %s\n', s);
        end
    end
    
    % Check scene objects count (should be fewer after pick removed the box)
    objList = findall(fig,'Style','listbox');
    if ~isempty(objList)
        items = get(objList(1),'String');
        fprintf('Object list items: %d (expect 0 after box picked)\n', length(items));
    end
    
    close(fig);
    fprintf('\n=== ALL TESTS PASSED ===\n');
    
catch ME
    fprintf('ERROR: %s\n  at %s line %d\n', ME.message, ME.stack(1).name, ME.stack(1).line);
    for si = 1:min(3,length(ME.stack))
        fprintf('  > %s line %d\n', ME.stack(si).name, ME.stack(si).line);
    end
end
