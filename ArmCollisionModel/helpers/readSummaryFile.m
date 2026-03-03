function summary = readSummaryFile(filepath)
%READSUMMARYFILE 解析 key:value 格式的summary文件
    summary = struct();
    if ~exist(filepath,'file'), return; end
    fid = fopen(filepath, 'r');
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line) || isempty(strtrim(line)) || line(1)=='#', continue; end
        tokens = regexp(line, '^\s*([a-zA-Z_0-9]+)\s*:\s*(.+)$', 'tokens');
        if ~isempty(tokens)
            key = strtrim(tokens{1}{1});
            val = strtrim(tokens{1}{2});
            summary.(key) = val;
        end
    end
    fclose(fid);
end
