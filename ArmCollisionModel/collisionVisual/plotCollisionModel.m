function outputStruct = plotCollisionModel(q, params, toolparams)
    global T6T;

    global d1 d4 d6 a2;
    d1 = params.DH.d1; 
    d4 = params.DH.d4; 
    d6 = params.DH.d6; 
    a2 = params.DH.a2;

    % % 设置ToolInFlange
    % toolRadius = 0.1;
    T6T = rpy2Rotation(0, 0, 0.1, 0, 0, 0);
    
    % 设置当前关节角
    % q = [0  pi/3  pi/2  pi  pi/2  0]; % 工具和基座刚好碰撞
    [T00, T01, T02, T03, T04, T05, T0T] = FK(q);
    Tf_tree = {T00, T01, T02, T03, T04, T05, T0T};
    
    % outputStruct = struct();
    outputStruct = plotSelfCollisonModel(Tf_tree, params, toolparams);
end

