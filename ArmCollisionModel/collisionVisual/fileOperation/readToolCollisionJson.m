function toolData = readToolCollisionJson(jsonFilePath)
    % Read JSON file
    jsonString = fileread(jsonFilePath);
    
    % Decode JSON string
    jsonData = jsondecode(jsonString);
    
    % Extract data from JSON and assign to toolData
    toolData.toolNum = jsonData.toolNum;
    
    for i = 1:toolData.toolNum
        toolKey = sprintf('tool%d', i);
        toolData.(toolKey).start = jsonData.(toolKey).start;
        toolData.(toolKey).end = jsonData.(toolKey).end;
        toolData.(toolKey).radius = jsonData.(toolKey).radius;
        toolData.(toolKey).type = jsonData.(toolKey).type;
        toolData.(toolKey).referenceFrame = jsonData.(toolKey).referenceFrame;
    end
end
