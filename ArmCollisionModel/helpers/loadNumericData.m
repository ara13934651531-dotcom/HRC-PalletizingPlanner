function data = loadNumericData(filepath)
%LOADNUMERICDATA 加载空格分隔数值文件 (跳过#注释和空行)
    fid = fopen(filepath, 'r');
    if fid==-1, error('Cannot open: %s', filepath); end
    lines = {};
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line) && ~isempty(strtrim(line)) && line(1)~='#'
            lines{end+1} = line; %#ok<AGROW>
        end
    end
    fclose(fid);
    nLines = length(lines);
    if nLines==0, data=[]; return; end
    vals = sscanf(lines{1}, '%f');
    nCols = length(vals);
    data = zeros(nLines, nCols);
    data(1,:) = vals';
    for i = 2:nLines
        vals = sscanf(lines{i}, '%f');
        if length(vals)==nCols, data(i,:)=vals'; end
    end
end
