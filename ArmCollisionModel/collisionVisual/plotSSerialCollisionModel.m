function outputStruct = plotSSerialCollisionModel(q, params, toolparams)
    global T6T;

    global d1 d2 d3 d4 d5 d6 a2 a3;
    d1 = params.DH.d1; 
    d2 = params.DH.d2;
    d3 = params.DH.d3;
    d4 = params.DH.d4; 
    d5 = params.DH.d5; 
    d6 = params.DH.d6; 
    a2 = -params.DH.a2;
    a3 = -params.DH.a3;

    % % 设置ToolInFlange
    % toolRadius = 0.1;
    T6T = rpy2Rotation(0, 0, 0, 0, 0, 0);
    
    % 设置当前关节角
    % q = [0  pi/3  pi/2  pi  pi/2  0]; % 工具和基座刚好碰撞
    [T00, T01, T02, T03, T04, T05, T0T] = FK_SSerial(q);
    Tf_tree = {T00, T01, T02, T03, T04, T05, T0T};

    for i = 1:length(Tf_tree)
        plotframe(Tf_tree{i}, 0.1, true);
        position = Tf_tree{i}(1:3, 4);  % 获取位置
        % 在位置旁打印 Ti
        text(position(1), position(2), position(3) + 0.1, ['T' num2str(i)], 'FontSize', 12, 'Color', 'red');
    end
    
    % outputStruct = struct();
    outputStruct = plotSelfCollisonModel(Tf_tree, params, toolparams);

end

