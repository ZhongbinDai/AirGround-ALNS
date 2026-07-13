function ensureDirectory(filePath)
    % 确保文件路径中的目录存在
    % 如果目录不存在，则创建它
    [fileDir, ~, ~] = fileparts(filePath);
    
    if ~isempty(fileDir) && ~exist(fileDir, 'dir')
        mkdir(fileDir);
        fprintf('目录已创建: %s\n', fileDir);
    end
end
