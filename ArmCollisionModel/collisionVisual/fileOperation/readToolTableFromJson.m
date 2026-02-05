function dataStruct = readToolTableFromJson(jsonFilePath)
    
    jsonString = fileread(jsonFilePath);
    % 使用 jsondecode 将 JSON 字符串解码为结构体
    jsonData = jsondecode(jsonString);

    % 从 JSON 结构中提取相应的字段
    toolCapsuleStartPoint = jsonData.coll_info.tool_capsule.start_point;
    toolCapsuleEndPoint = jsonData.coll_info.tool_capsule.end_point;
    toolCapsuleRadius = jsonData.coll_info.tool_capsule.radius;

    tableLozengeOffset = jsonData.coll_info.table_lozenge.offset;
    tableLozengeLWH = jsonData.coll_info.table_lozenge.lwh;
    tableLozengeRadius = jsonData.coll_info.table_lozenge.radius;

    toolTCP = jsonData.robot_info.tool_tcp;
    dh = jsonData.robot_info.dh;
    jointPositions = jsonData.robot_info.joint_positions;

    % 将提取的数据保存到结构体中
    dataStruct.toolCapsule.start_point = toolCapsuleStartPoint;
    dataStruct.toolCapsule.end_point = toolCapsuleEndPoint;
    dataStruct.toolCapsule.radius = toolCapsuleRadius;

    dataStruct.tableLozenge.offset = tableLozengeOffset;
    dataStruct.tableLozenge.lwh = tableLozengeLWH;
    dataStruct.tableLozenge.radius = tableLozengeRadius;

    dataStruct.toolTCP = toolTCP;
    dataStruct.dh = dh;
    dataStruct.joint_positions = jointPositions;
end