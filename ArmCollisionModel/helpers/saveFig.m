function saveFig(fig, outputDir, name)
%SAVEFIG_V15 保存figure为PNG + .fig文件
    fn = fullfile(outputDir, [name '.png']);
    print(fig, fn, '-dpng', '-r150');
    fprintf('  Saved: %s\n', fn);
    % 保存 .fig 文件 (交互式3D查看)
    fnFig = fullfile(outputDir, [name '.fig']);
    savefig(fig, fnFig, 'compact');
    fprintf('  Saved: %s\n', fnFig);
end
