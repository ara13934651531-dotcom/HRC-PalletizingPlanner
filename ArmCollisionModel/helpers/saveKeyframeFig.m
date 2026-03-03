function saveKeyframeFig(fig, keyframeDir, name, description)
%SAVEKEYFRAMEFIG 保存动画关键帧为独立.fig+.png快照
%   用户可在MATLAB中 openfig('xxx.fig') 进行3D旋转/缩放/平移交互查看
%   fig:          源figure
%   keyframeDir:  输出目录
%   name:         文件名 (不含扩展名)
%   description:  日志描述
    try
        % 保存.fig (compact模式, 减小文件大小)
        fnFig = fullfile(keyframeDir, [name '.fig']);
        savefig(fig, fnFig, 'compact');
        % 同时保存.png (150dpi, 便于快速预览)
        fnPng = fullfile(keyframeDir, [name '.png']);
        print(fig, fnPng, '-dpng', '-r150');
        fprintf('    关键帧: %s → .fig+.png (%s)\n', name, description);
    catch ME
        fprintf('    [WARN] 关键帧保存失败 %s: %s\n', name, ME.message);
    end
end
