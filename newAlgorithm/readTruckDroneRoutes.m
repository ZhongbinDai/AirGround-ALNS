function [truckRoutes, droneRoutes] = readTruckDroneRoutes(fileName)
	% 读取 truck_routes 与 drone_routes*
	truckRoutes = {};
	droneRoutes = {}; % 作为每架无人机的若干次飞行（元胞数组）

	fid = fopen(fileName, 'r');
	assert(fid ~= -1, '无法打开结果文件: %s', fileName);
	cl = onCleanup(@() fclose(fid));

	while true
		line = fgetl(fid);
		if ~ischar(line), break; end
		line = strtrim(line);
		if isempty(line), continue; end

		% name = ..., value = ...
		eqPos = strfind(line, '=');
		assert(~isempty(eqPos), '结果文件格式错误(缺少=): %s', line);
		name = strtrim(line(1:eqPos(1)-1));
		valueStr = strtrim(line(eqPos(1)+1:end));

		% 尝试按 JSON 解析（与 Python 列表基本一致）
		try
			value = jsondecode(valueStr);
		catch
			error('无法解析为 JSON: %s', valueStr);
		end

		if strcmp(name, 'truck_route')
			% value 为 cell 或 double[][]；统一为 cell，每个元素为一条卡车路径(0基节点ID)
			if isnumeric(value)
				% 可能是规则矩阵，拆成行
				rows = size(value,1);
				tmp = cell(1, rows);
				for r = 1:rows
					rowVec = value(r, :);
					rowVec = rowVec(~isnan(rowVec));
					tmp{r} = value(r, :);
				end
				truckRoutes = tmp;
			elseif iscell(value)
				truckRoutes = value;
			else
				error('未支持的 truck_route 解析类型');
			end
		elseif startsWith(name, 'drone_routes')
			% value 为该无人机的若干次飞行（list of lists），统一 cell-of-cell
			if isnumeric(value)
				% 规则矩阵 => 每行一次飞行，且各行长度可能不一致时 jsondecode 会转 cell
				if isvector(value)
					droneRoutes{end+1} = {value}; 
				else
					tmp = cell(1, size(value,1));
					for r = 1:size(value,1)
						tmp{r} = value(r, :);
					end
					droneRoutes{end+1} = tmp; 
				end
			elseif iscell(value)
				% 元素也可能是 cell 或 double
				droneRoutes{end+1} = value; %#ok<AGROW>
			else
				error('未支持的 drone_routes 解析类型');
			end
		end
    end
end

