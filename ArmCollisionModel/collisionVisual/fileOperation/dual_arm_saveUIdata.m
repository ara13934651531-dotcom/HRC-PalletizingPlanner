function dual_arm_saveUIdata(q, outputStruct, params, json_filename)
%SAVEHIDATA 此处显示有关此函数的摘要
%left
    paramOut.l.inputJoint = q*180/pi;
    paramOut.l.base.start = outputStruct.l.base_bc1';
    paramOut.l.base.end = outputStruct.l.base_bc2';
    paramOut.l.base.type = params.base.type;
    paramOut.l.base.radius = params.base.radius;
    paramOut.l.lowerArm.start = outputStruct.l.lowerArm_la1';
    paramOut.l.lowerArm.end = outputStruct.l.lowerArm_la2';
    paramOut.l.lowerArm.type = params.l.lowerArm.type;
    paramOut.l.lowerArm.radius = params.l.lowerArm.radius;
    paramOut.l.elbow.start = outputStruct.l.elbow_e1';
    paramOut.l.elbow.end = outputStruct.l.elbow_e2';
    paramOut.l.elbow.type = params.l.elbow.type;
    paramOut.l.elbow.radius = params.l.elbow.radius;
    paramOut.l.upperArm.start = outputStruct.l.upperArm_ua1';
    paramOut.l.upperArm.end = outputStruct.l.upperArm_ua2';
    paramOut.l.upperArm.type = params.l.upperArm.type;
    paramOut.l.upperArm.radius = params.l.upperArm.radius;
    paramOut.l.wrist.offset = outputStruct.l.wrist_wc;
    paramOut.l.wrist.type = params.l.wrist.type;
    paramOut.l.wrist.radius = params.l.wrist.radius;

    for i = 1:outputStruct.l.toolNum
        toolKey = sprintf('tool%d', i);
        paramOut.l.(toolKey).type = outputStruct.(toolKey).type;
        if outputStruct.(toolKey).type == "capsule"
            paramOut.l.(toolKey).start = outputStruct.l.(toolKey).t_p1';
            paramOut.l.(toolKey).end = outputStruct.l.(toolKey).t_p2';
            paramOut.l.(toolKey).radius = outputStruct.l.(toolKey).radius;
        end
    end

    % 将 JSON 数据写入文件
    % json_filename = './data/output1.json';
    
    savejson('', paramOut.l, json_filename);

%right
 paramOut.r.inputJoint = q*180/pi;
    %paramOut.r.base.start = outputStruct.r.base_bc1';
    %paramOut.r.base.end = outputStruct.r.base_bc2';
    %paramOut.r.base.type = params.base.type;
    %paramOut.r.base.radius = params.base.radius;
    paramOut.r.lowerArm.start = outputStruct.r.lowerArm_la1';
    paramOut.r.lowerArm.end = outputStruct.r.lowerArm_la2';
    paramOut.r.lowerArm.type = params.r.lowerArm.type;
    paramOut.r.lowerArm.radius = params.r.lowerArm.radius;
    paramOut.r.elbow.start = outputStruct.r.elbow_e1';
    paramOut.r.elbow.end = outputStruct.r.elbow_e2';
    paramOut.r.elbow.type = params.r.elbow.type;
    paramOut.r.elbow.radius = params.r.elbow.radius;
    paramOut.r.upperArm.start = outputStruct.r.upperArm_ua1';
    paramOut.r.upperArm.end = outputStruct.r.upperArm_ua2';
    paramOut.r.upperArm.type = params.r.upperArm.type;
    paramOut.r.upperArm.radius = params.r.upperArm.radius;
    paramOut.r.wrist.offset = outputStruct.r.wrist_wc;
    paramOut.r.wrist.type = params.r.wrist.type;
    paramOut.r.wrist.radius = params.r.wrist.radius;

    for i = 1:outputStruct.l.toolNum
        toolKey = sprintf('tool%d', i);
        paramOut.r.(toolKey).type = outputStruct.r.(toolKey).type;
        if outputStruct.(toolKey).type == "capsule"
            paramOut.r.(toolKey).start = outputStruct.r.(toolKey).t_p1';
            paramOut.r.(toolKey).end = outputStruct.r.(toolKey).t_p2';
            paramOut.r.(toolKey).radius = outputStruct.r.(toolKey).radius;
        end
    end






end

