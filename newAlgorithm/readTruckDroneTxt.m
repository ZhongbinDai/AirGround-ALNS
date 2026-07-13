function [data1, data2] = readTruckDroneTxt(filename)
	lines = strtrim(readlines(filename));
	if isempty(lines)
		error('文件为空：%s', filename);
	end

	idx = 1;
	function l = nextDataLine()
		% 跳过注释行与空行，返回下一条数据行字符串
		while idx <= numel(lines)
			cur = lines(idx);
			idx = idx + 1;
			if strlength(cur) == 0
				continue;
			end
			if startsWith(cur, "/*")
				% 注释行，继续
				continue;
			end
			l = cur;
			return;
		end
		l = string(missing);
	end

	% 依次读取头部参数（按文件固有顺序）
	numDronesPerTruck = str2double(nextDataLine());
	numTrucks         = str2double(nextDataLine());
	truckCapacity     = str2double(nextDataLine());
	droneCapacity     = str2double(nextDataLine());
	truckMaxDistance  = str2double(nextDataLine());
	droneMaxDistance  = str2double(nextDataLine());
	truckSpeed        = str2double(nextDataLine());
	droneSpeed        = str2double(nextDataLine());
	numNodes          = double(str2double(nextDataLine()));

	% 读取 Depot 行（x y name）
	% 跳过可能存在的注释标题行
	depotLine = nextDataLine();
	while strlength(depotLine) == 0 || startsWith(depotLine, "/*")
		depotLine = nextDataLine();
	end
	parts = textscan(depotLine, '%f %f %s', 'Delimiter', ' ');
	if numel(parts) < 3 || any(cellfun(@isempty, parts(1:2))) || isempty(parts{3})
		error('Depot 行解析失败：%s', depotLine);
	end
	depot = struct('x', parts{1}(1), 'y', parts{2}(1), 'name', string(parts{3}{1}));

	% 读取 Locations，直到文件结束（x y name demand）
	locX = [];
	locY = [];
	locName = strings(0,1);
	locDemand = [];

	% 跳过 "Locations" 标题注释行（若当前行是注释）
	% 如果 depot 后紧跟的是数据，也能被下面循环解析
	while true
		if idx > numel(lines)
			break;
		end
		line = lines(idx);
		if strlength(strtrim(line)) == 0 || startsWith(strtrim(line), "/*")
			idx = idx + 1;
			continue;
		end
		% 尝试解析为 2 实数 + 文本 + 1 实数
		parts = textscan(line, '%f %f %s %f', 'Delimiter', ' ');
		if isempty(parts{1}) || isempty(parts{2}) || isempty(parts{3}) || isempty(parts{4})
			% 遇到非数据行则跳过
			idx = idx + 1;
			continue;
		end
		locX(end+1,1) = parts{1}(1);
		locY(end+1,1) = parts{2}(1);
		locName(end+1,1) = string(parts{3}{1});
		locDemand(end+1,1) = parts{4}(1);
		idx = idx + 1;
	end

	locations = table(locX, locY, locName, locDemand, ...
		'VariableNames', {'x','y','name','demand'});

	S = struct();
	S.meta = struct( ...
		'numDronesPerTruck', numDronesPerTruck, ...
		'numTrucks',         numTrucks, ...
		'truckCapacity',     truckCapacity, ...
		'droneCapacity',     droneCapacity, ...
		'truckMaxDistance',  truckMaxDistance, ...
		'droneMaxDistance',  droneMaxDistance, ...
		'truckSpeed',        truckSpeed, ...
		'droneSpeed',        droneSpeed, ...
		'numNodes',          numNodes ...
	);
	S.depot = depot;
	S.locations = locations;
	S.file = string(filename);
	S.summary = struct( ...
		'numCustomers', height(locations), ...
		'hasDepot',     ~isempty(depot) ...
	);

    data1 = [S.locations.x, S.locations.y, S.locations.demand; ...  % 客户点：x, y, demand
         S.depot.x, S.depot.y, 0];                              % 供应点：x, y, 0

    % 构建 data2 向量：参数数组
    data2 = [S.meta.numDronesPerTruck, ...
             S.meta.numTrucks, ...
             S.meta.truckCapacity, ...
             S.meta.droneCapacity, ...
             S.meta.truckMaxDistance, ...
             S.meta.droneMaxDistance, ...
             S.meta.truckSpeed, ...
             S.meta.droneSpeed];

end


