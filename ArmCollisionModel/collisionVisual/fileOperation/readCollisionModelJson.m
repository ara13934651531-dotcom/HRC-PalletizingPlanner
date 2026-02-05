function configStruct = readCollisionModelJson(jsonFilePath)
    % Read JSON file
    jsonString = fileread(jsonFilePath);
    
    % Decode JSON string
    jsonData = jsondecode(jsonString);
    
    % Extract data from JSON and assign to configStruct
    configStruct.RobType = jsonData.RobType;
    configStruct.DH = jsonData.DH;

    
    configStruct.base.start = jsonData.base.start;
    configStruct.base.end = jsonData.base.end;
    configStruct.base.radius = jsonData.base.radius;
    configStruct.base.type = jsonData.base.type;
    configStruct.base.referenceFrame = jsonData.base.referenceFrame;
    
    configStruct.lowerArm.start = jsonData.lowerArm.start;
    configStruct.lowerArm.end = jsonData.lowerArm.end;
    configStruct.lowerArm.radius = jsonData.lowerArm.radius;
    configStruct.lowerArm.type = jsonData.lowerArm.type;
    configStruct.lowerArm.referenceFrame = jsonData.lowerArm.referenceFrame;
    
    configStruct.elbow.start = jsonData.elbow.start;
    configStruct.elbow.end = jsonData.elbow.end;
    configStruct.elbow.radius = jsonData.elbow.radius;
    configStruct.elbow.type = jsonData.elbow.type;
    configStruct.elbow.referenceFrame = jsonData.elbow.referenceFrame;
    
    configStruct.upperArm.start = jsonData.upperArm.start;
    configStruct.upperArm.end = jsonData.upperArm.end;
    configStruct.upperArm.radius = jsonData.upperArm.radius;
    configStruct.upperArm.type = jsonData.upperArm.type;
    configStruct.upperArm.referenceFrame = jsonData.upperArm.referenceFrame;
    
    configStruct.wrist.offset = jsonData.wrist.offset;
    configStruct.wrist.radius = jsonData.wrist.radius;
    configStruct.wrist.type = jsonData.wrist.type;
    configStruct.wrist.referenceFrame = jsonData.wrist.referenceFrame;
end
