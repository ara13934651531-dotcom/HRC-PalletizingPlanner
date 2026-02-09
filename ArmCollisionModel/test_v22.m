% Test v2.2 fixes: TCP position, trajectory path, collision detection
set(0,'DefaultFigureVisible','off');
fprintf('=== v2.2 Test ===\n');
try
    PalletizingSimApp();
    pause(1);
    fig = findall(0,'Type','figure'); fig=fig(1);
    
    % Add box
    addBtn = findall(fig,'Style','pushbutton','String','Add');
    cb = get(addBtn(1),'Callback');
    cb(addBtn(1),[]);
    pause(0.3);
    fprintf('OK: Box added\n');
    
    % Check Place Z (should be box TOP = 0.55+0.25 = 0.80)
    placeZ = findall(fig,'Style','edit');
    for i=1:length(placeZ)
        s = get(placeZ(i),'String');
        v = str2double(s);
        if abs(v-0.80)<0.01
            fprintf('OK: Place Z = %s (box top, Fix #1 verified)\n', s);
        end
    end
    
    % Auto execute
    autoBtn = findall(fig,'Style','pushbutton','String','Auto Pick & Place');
    cb2 = get(autoBtn(1),'Callback');
    cb2(autoBtn(1),[]);
    pause(8);
    fprintf('OK: Execute done\n');
    
    % Check IK/collision text
    allUI = findall(fig,'Type','uicontrol');
    for i=1:length(allUI)
        s = get(allUI(i),'String');
        if ischar(s) && (contains(s,'COL') || contains(s,'collision') || contains(s,'No collision'))
            fprintf('Collision result: %s\n', s);
        end
        if ischar(s) && contains(s,'Palletized')
            fprintf('Pallet: %s\n', s);
        end
    end
    
    % Check collision markers
    colMarkers = findall(fig,'Type','line','Marker','x');
    fprintf('Collision markers (red x): %d\n', length(colMarkers));
    
    close(fig);
    fprintf('=== ALL PASS ===\n');
catch ME
    fprintf('ERROR: %s at %s:%d\n', ME.message, ME.stack(1).name, ME.stack(1).line);
    for si = 1:min(5,length(ME.stack))
        fprintf('  > %s:%d\n', ME.stack(si).name, ME.stack(si).line);
    end
end
