function val = getField(s, fieldName, default)
%GETFIELD_SAFE 安全获取结构体字段 (不存在时返回默认值)
    if isfield(s, fieldName), val = s.(fieldName); else, val = default; end
end
