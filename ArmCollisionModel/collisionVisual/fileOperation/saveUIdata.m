function saveUIdata(q, outputStruct, params, json_filename)
%SAVEHIDATA 此处显示有关此函数的摘要
%   此处显示详细说明
    paramOut.inputJoint = q*180/pi;
    paramOut.base.start = outputStruct.base_bc1';
    paramOut.base.end = outputStruct.base_bc2';
    paramOut.base.type = params.base.type;
    paramOut.base.radius = params.base.radius;
    paramOut.lowerArm.start = outputStruct.lowerArm_la1';
    paramOut.lowerArm.end = outputStruct.lowerArm_la2';
    paramOut.lowerArm.type = params.lowerArm.type;
    paramOut.lowerArm.radius = params.lowerArm.radius;
    paramOut.elbow.start = outputStruct.elbow_e1';
    paramOut.elbow.end = outputStruct.elbow_e2';
    paramOut.elbow.type = params.elbow.type;
    paramOut.elbow.radius = params.elbow.radius;
    paramOut.upperArm.start = outputStruct.upperArm_ua1';
    paramOut.upperArm.end = outputStruct.upperArm_ua2';
    paramOut.upperArm.type = params.upperArm.type;
    paramOut.upperArm.radius = params.upperArm.radius;
    paramOut.wrist.offset = outputStruct.wrist_wc;
    paramOut.wrist.type = params.wrist.type;
    paramOut.wrist.radius = params.wrist.radius;

    for i = 1:outputStruct.toolNum
        toolKey = sprintf('tool%d', i);
        paramOut.(toolKey).type = outputStruct.(toolKey).type;
        if outputStruct.(toolKey).type == "capsule"
            paramOut.(toolKey).start = outputStruct.(toolKey).t_p1';
            paramOut.(toolKey).end = outputStruct.(toolKey).t_p2';
            paramOut.(toolKey).radius = outputStruct.(toolKey).radius;
        end
    end

    % 将 JSON 数据写入文件
    % json_filename = './data/output1.json';
    
    savejson('', paramOut, json_filename);
end

