function outputStruct = plotToolCollisonModel(Tf_tree, params)
    % 定义输出结构体
    
view(3)

    T0T = Tf_tree{7};
 hold on;
    % 绘制基座部分
    [bc1, bc2] = plotCapsule(T00, params.base.start(1), params.base.start(2), params.base.start(3), ...
        params.base.end(1), params.base.end(2), params.base.end(3), params.base.radius);
    hold on;
    outputStruct.base_bc1 = bc1;
    outputStruct.base_bc2 = bc2;

    % 绘制低臂部分
    [la1, la2] = plotCapsule(T02, params.lowerArm.start(1), params.lowerArm.start(2), params.lowerArm.start(3), ...
        params.lowerArm.end(1), params.lowerArm.end(2), params.lowerArm.end(3), params.lowerArm.radius);
    outputStruct.lowerArm_la1 = la1;
    outputStruct.lowerArm_la2 = la2;
 hold on;
    % 绘制肘部分
    [e1, e2] = plotCapsule(T03, params.elbow.start(1), params.elbow.start(2), params.elbow.start(3), ...
        params.elbow.end(1), params.elbow.end(2), params.elbow.end(3), params.elbow.radius);
    outputStruct.elbow_e1 = e1;
    outputStruct.elbow_e2 = e2;
 hold on;
    % 绘制上臂部分
    [ua1, ua2] = plotCapsule(T04, params.upperArm.start(1), params.upperArm.start(2), params.upperArm.start(3), ...
        params.upperArm.end(1), params.upperArm.end(2), params.upperArm.end(3), params.upperArm.radius);
    outputStruct.upperArm_ua1 = ua1;
    outputStruct.upperArm_ua2 = ua2;
 hold on;
    % 绘制腕部分
    wc = plotBall(T05, params.wrist.offset(1), params.wrist.offset(2), params.wrist.offset(3), params.wrist.radius);
    outputStruct.wrist_wc = wc;
 hold on;
    % 绘制工具部分
    toolRadius = 0.06;
    [tool1, tool2] = plotCapsule(T0T, 0, 0, -0.02, 0, 0, -0.16, toolRadius);
    outputStruct.tool_tool1 = tool1;
    outputStruct.tool_tool2 = tool2;
 hold on;
    % % 绘制坐标系
    % outputStruct.frame_T04 = plotframe(T04, 0.1, false);
    % outputStruct.frame_T0T = plotframe(T0T, 0.1, false);
    % outputStruct.frame_T00 = plotframe(T00, 0.1, false);
end

