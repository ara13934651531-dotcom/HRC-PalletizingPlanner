function configStruct = read_dual_arm_ModelJson(jsonFilePath)
    % 读取JSON文件
    jsonString = fileread(jsonFilePath);
    
    % 解析JSON字符串
    jsonData = jsondecode(jsonString);
    
    % 提取数据并赋值到configStruct
    configStruct.RobType = jsonData.RobType;
    configStruct.DH = jsonData.DH;  % DH已包含l和r子结构
    
    % 基座参数（共用部分）
    configStruct.base.start = jsonData.base.start;
    configStruct.base.end = jsonData.base.end;
    configStruct.base.radius = jsonData.base.radius;
    configStruct.base.type = jsonData.base.type;
    configStruct.base.referenceFrame = jsonData.base.referenceFrame;
    
    % 左臂参数
    configStruct.l.lowerArm.start = jsonData.l_lowerArm.start;
    configStruct.l.lowerArm.end = jsonData.l_lowerArm.end;
    configStruct.l.lowerArm.radius = jsonData.l_lowerArm.radius;
    configStruct.l.lowerArm.type = jsonData.l_lowerArm.type;
    configStruct.l.lowerArm.referenceFrame = jsonData.l_lowerArm.referenceFrame;
    
    configStruct.l.elbow.start = jsonData.l_elbow.start;
    configStruct.l.elbow.end = jsonData.l_elbow.end;
    configStruct.l.elbow.radius = jsonData.l_elbow.radius;
    configStruct.l.elbow.type = jsonData.l_elbow.type;
    configStruct.l.elbow.referenceFrame = jsonData.l_elbow.referenceFrame;
    
    configStruct.l.upperArm.start = jsonData.l_upperArm.start;
    configStruct.l.upperArm.end = jsonData.l_upperArm.end;
    configStruct.l.upperArm.radius = jsonData.l_upperArm.radius;
    configStruct.l.upperArm.type = jsonData.l_upperArm.type;
    configStruct.l.upperArm.referenceFrame = jsonData.l_upperArm.referenceFrame;
    
    configStruct.l.wrist.offset = jsonData.l_wrist.offset;
    configStruct.l.wrist.radius = jsonData.l_wrist.radius;
    configStruct.l.wrist.type = jsonData.l_wrist.type;
    configStruct.l.wrist.referenceFrame = jsonData.l_wrist.referenceFrame;
    
    % 右臂参数
    configStruct.r.lowerArm.start = jsonData.r_lowerArm.start;
    configStruct.r.lowerArm.end = jsonData.r_lowerArm.end;
    configStruct.r.lowerArm.radius = jsonData.r_lowerArm.radius;
    configStruct.r.lowerArm.type = jsonData.r_lowerArm.type;
    configStruct.r.lowerArm.referenceFrame = jsonData.r_lowerArm.referenceFrame;
    
    configStruct.r.elbow.start = jsonData.r_elbow.start;
    configStruct.r.elbow.end = jsonData.r_elbow.end;
    configStruct.r.elbow.radius = jsonData.r_elbow.radius;
    configStruct.r.elbow.type = jsonData.r_elbow.type;
    configStruct.r.elbow.referenceFrame = jsonData.r_elbow.referenceFrame;
    
    configStruct.r.upperArm.start = jsonData.r_upperArm.start;
    configStruct.r.upperArm.end = jsonData.r_upperArm.end;
    configStruct.r.upperArm.radius = jsonData.r_upperArm.radius;
    configStruct.r.upperArm.type = jsonData.r_upperArm.type;
    configStruct.r.upperArm.referenceFrame = jsonData.r_upperArm.referenceFrame;
    
    configStruct.r.wrist.offset = jsonData.r_wrist.offset;
    configStruct.r.wrist.radius = jsonData.r_wrist.radius;
    configStruct.r.wrist.type = jsonData.r_wrist.type;
    configStruct.r.wrist.referenceFrame = jsonData.r_wrist.referenceFrame;

    %人形外观参数
    configStruct.truck.length = jsonData.truck.length;
    configStruct.truck.width = jsonData.truck.width;
    configStruct.truck.height= jsonData.truck.height;
    configStruct.truck.offset = jsonData.truck.offset;
    configStruct.truck.radius = jsonData.truck.radius;

    %头部包络
    configStruct.head.length = jsonData.head.length;
    configStruct.head.width = jsonData.head.width;
    configStruct.head.height = jsonData.head.height;
    configStruct.head.offset = jsonData.head.offset;
    configStruct.head.radius = jsonData.head.radius;

    %平台包络
    configStruct.plane.length = jsonData.plane.length;
    configStruct.plane.width = jsonData.plane.width;
    configStruct.plane.height = jsonData.plane.height;
    configStruct.plane.offset = jsonData.plane.offset;
    configStruct.plane.radius = jsonData.plane.radius;

end