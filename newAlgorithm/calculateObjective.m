function [maxFinishTime, G, deg] = calculateObjective(resultFilename, data)
    [truckRoutes, droneRoutes] = readTruckDroneRoutes(resultFilename);
    

	% 构图（邻接表：G{i} 为 [b, tau; ...]）
	G = cell(data.N, 1);
	deg = zeros(data.N, 1);
	c   = zeros(data.N, 1); % 每个节点的无人机载重累加（如需用）

	% 卡车边（truck_routes 中节点为0基，这里转1基）
	for k = 1:numel(truckRoutes)
		route0 = truckRoutes{k};
		if isempty(route0), continue; end
		for i = 1:(numel(route0)-1)
			a = route0(i)   + 1;
			b = route0(i+1) + 1;
			tau = data.tau_t(a, b);
			G{a} = [G{a}; b, tau];
			deg(b) = deg(b) + 1;
		end
	end

	% 无人机边
	for d = 1:numel(droneRoutes)
		runs = droneRoutes{d};
		if isempty(runs), continue; end

		k0 = floor((d-1) / data.N_d); % 0 基卡车编号
		k  = k0 + 1;

		for r = 1:numel(runs)
			seq = runs{r};
			if isempty(seq), continue; end
			seq = seq(:)'; 

			a0 = truckRoutes{k}( seq(1) + 1 );
			a  = a0 + 1; 

			if numel(seq) == 2
				b0 = truckRoutes{k}( seq(end) + 1 );
            else
				b0 = seq(2);
			end
			b = b0 + 1;

			tau = data.tau_d(a, b);
			G{a} = [G{a}; b, tau];
			deg(b) = deg(b) + 1;
			c(b) = c(a) + data.w(b);

			% 中间段（若存在）：按无人机序列中的相邻节点连边
			for i = 2:(numel(seq)-2)
				a0 = seq(i);
				b0 = seq(i+1);
				a  = a0 + 1;
				b  = b0 + 1;
				tau = data.tau_d(a, b);
				G{a} = [G{a}; b, tau];
				deg(b) = deg(b) + 1;
				c(b) = c(a) + data.w(b);
			end

			% 最后一段：返回卡车（若序列长度 > 2）
			if numel(seq) ~= 2
				a0 = seq(end-1);
				b0 = truckRoutes{k}( seq(end) + 1 );
				a  = a0 + 1;
				b  = b0 + 1;
				tau = data.tau_d(a, b);
				G{a} = [G{a}; b, tau];
				deg(b) = deg(b) + 1;
			end
		end
	end

	% 拓扑排序，计算每个点的最早出发时间 t_d
	t_d = zeros(data.N, 1);
	queue = zeros(data.N, 1);
	head = 1; tail = 0;

	% depot 假设为 0 基 => 1 基索引为 1
	tail = tail + 1;
	queue(tail) = 1;

    degTemp = deg;
	while head <= tail
		a = queue(head);
        head = head + 1;
        % fprintf('%d %d\n', head, a);

		edges = G{a};
		if ~isempty(edges)
			for idx = 1:size(edges,1)
				b   = edges(idx, 1);
				tau = edges(idx, 2);
				t_d(b) = max(t_d(b), t_d(a) + tau);
				deg(b) = deg(b) - 1;
				if deg(b) == 0 && b ~= 1
					tail = tail + 1;
					queue(tail) = b;
				end
			end
		end
	end

    deg = degTemp;
	
    maxFinishTime = t_d(1);
end




