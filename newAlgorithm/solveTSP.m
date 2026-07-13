function path = solveTSP(coordinates)
% solveTSP - 使用整数线性规划求解TSP问题
% 输入:
%   coordinates - n行2列的矩阵，每行代表一个点的坐标 [x, y]
% 输出:
%   path - 路径序列，格式为 [1,2,3,...,n,1]
%
% 算法说明:
%   使用整数线性规划方法，通过逐步消除子回路来求解TSP问题
%   距离计算采用欧氏距离公式: sqrt((x2-x1)^2 + (y2-y1)^2)
%   适用于小到中等规模的TSP问题

    % 检查输入参数
    if nargin < 1
        error('需要提供坐标矩阵作为输入参数');
    end
    
    if size(coordinates, 2) ~= 2
        error('坐标矩阵必须是n行2列的格式');
    end
    
    nStops = size(coordinates, 1);
    if nStops < 3
        error('至少需要3个点才能构成TSP问题');
    end
    
    % 提取坐标
    stopsLon = coordinates(:, 1);  % x坐标
    stopsLat = coordinates(:, 2);  % y坐标
    
    % 计算所有点对之间的欧氏距离
    idxs = nchoosek(1:nStops, 2);
    % 欧氏距离公式: sqrt((x2-x1)^2 + (y2-y1)^2)
    dist = sqrt((stopsLat(idxs(:,1)) - stopsLat(idxs(:,2))).^2 + ...
                (stopsLon(idxs(:,1)) - stopsLon(idxs(:,2))).^2);

    % dist = abs(stopsLat(idxs(:,1)) - stopsLat(idxs(:,2))) + abs(stopsLon(idxs(:,1)) - stopsLon(idxs(:,2)));
    lendist = length(dist);
    
    % 创建图结构
    G = graph(idxs(:,1), idxs(:,2));
    
    % 设置优化问题
    tsp = optimproblem;
    trips = optimvar('trips', lendist, 1, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);
    
    % 目标函数：最小化总距离
    tsp.Objective = dist' * trips;
    
    % 约束条件：每个点必须有且仅有两条边相连
    constr2trips = optimconstr(nStops, 1);
    for stop = 1:nStops
        whichIdxs = outedges(G, stop);
        constr2trips(stop) = sum(trips(whichIdxs)) == 2;
    end
    tsp.Constraints.constr2trips = constr2trips;
    
    % 设置求解器选项
    opts = optimoptions('intlinprog', 'Display', 'off');
    
    % 求解TSP问题
    tspsol = solve(tsp, 'options', opts);
    
    % 处理解
    tspsol.trips = logical(round(tspsol.trips));
    Gsol = graph(idxs(tspsol.trips, 1), idxs(tspsol.trips, 2), [], numnodes(G));
    
    % 检查子回路
    tourIdxs = conncomp(Gsol);
    numtours = max(tourIdxs);
    
    % 消除子回路
    k = 1;
    while numtours > 1
        % 添加子回路约束
        for ii = 1:numtours
            inSubTour = (tourIdxs == ii);
            a = all(inSubTour(idxs), 2);
            constrname = "subtourconstr" + num2str(k);
            tsp.Constraints.(constrname) = sum(trips(a)) <= (nnz(inSubTour) - 1);
            k = k + 1;
        end
        
        % 重新求解
        tspsol = solve(tsp, 'options', opts);
        tspsol.trips = logical(round(tspsol.trips));
        Gsol = graph(idxs(tspsol.trips, 1), idxs(tspsol.trips, 2), [], numnodes(G));
        
        % 检查新的子回路数量
        tourIdxs = conncomp(Gsol);
        numtours = max(tourIdxs);
    end
    
    % 从图结构中提取路径
    path = extractPathFromGraph(Gsol, nStops);
    
    % 确保路径以起始点结束
    if path(end) ~= path(1)
        path = [path, path(1)];
    end
end

function path = extractPathFromGraph(G, nStops)
% extractPathFromGraph - 从图中提取路径序列
% 输入:
%   G - 图对象
%   nStops - 点的数量
% 输出:
%   path - 路径序列

    % 获取图的边
    edges = G.Edges.EndNodes;
    
    % 构建邻接表
    adjList = cell(nStops, 1);
    for i = 1:size(edges, 1)
        from = edges(i, 1);
        to = edges(i, 2);
        adjList{from} = [adjList{from}, to];
        adjList{to} = [adjList{to}, from];
    end
    
    % 从点1开始构建路径
    path = [];
    visited = false(nStops, 1);
    current = 1;
    
    while length(path) < nStops
        path = [path, current];
        visited(current) = true;
        
        % 找到下一个未访问的相邻点
        next = [];
        for neighbor = adjList{current}
            if ~visited(neighbor)
                next = neighbor;
                break;
            end
        end
        
        if isempty(next)
            % 如果没有未访问的相邻点，寻找任意未访问的点
            for i = 1:nStops
                if ~visited(i)
                    next = i;
                    break;
                end
            end
        end
        
        current = next;
    end
    
    % 添加起始点形成回路
    path = [path, path(1)];
end
