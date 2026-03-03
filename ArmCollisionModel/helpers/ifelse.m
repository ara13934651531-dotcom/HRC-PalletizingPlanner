function result = ifelse(cond, trueVal, falseVal)
%IFELSE 条件选择 (三元运算符替代)
    if cond, result = trueVal; else, result = falseVal; end
end
