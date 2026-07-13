function [maxFinishTime] = getMaxFinishTime(truckPathTable, realDronePathTableCell, model)
    [G, deg] = getDigraph(truckPathTable, realDronePathTableCell, model);

    n = size(model.timeMatOfTruck, 1);
    finishTimeArray = zeros(n, 1);                                          % t_d[u]：节点 u 的"最早出发/完成时间"（本质是所有前驱到达时间加上边权的最大值）

    queue = zeros(n, 1);
	head = 1;
    tail = 0;

    for i = model.numOfCustomer + 1: n
	    tail = tail + 1;
	    queue(tail) = i;
    end

    while head <= tail
		a = queue(head);
        head = head + 1;

		edges = G{a};
		if ~isempty(edges)
			for idx = 1: size(edges, 1)
				b   = edges(idx, 1);
				tau = edges(idx, 2);
				finishTimeArray(b) = max(finishTimeArray(b), finishTimeArray(a) + tau);
				deg(b) = deg(b) - 1;
				if deg(b) == 0 && b <= model.numOfCustomer
					tail = tail + 1;
					queue(tail) = b;
				end
			end
		end
    end

    maxFinishTime = max(finishTimeArray);
end