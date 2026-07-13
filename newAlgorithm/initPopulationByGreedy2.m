function [population] = initPopulationByGreedy2(populationSize, beta, greedyEpsilon, randTruck, model)
    probMatOfTruck = 1 ./ (model.distanceMatOfTruck .^ beta);
    probMatOfDrone = 1 ./ (model.distanceMatOfDrone .^ beta);

    numOfDecVariables = model.numOfDecVariables;
    population = zeros(populationSize, numOfDecVariables);
    for i = 1 : populationSize
        if rand() < greedyEpsilon
            population(i, :) = getIndividualByGreedy(greedyEpsilon, probMatOfTruck, probMatOfDrone, randTruck, model);
        else
            population(i, :) = model.initIndividual(model);
        end
    end
end

% 为卡车、无人机分配客户（满足容量约束）
function [pathTable] = allocateCustomer(randTruck, model)
    [sequence] = model.sequence;
    sequence(1: model.numOfCustomer) = [];
    [pathTable] = model.getPathTable(sequence, model);                          % 第1列卡车槽，其他列无人机槽
    for slotId = 1: numel(pathTable)
        path = pathTable{slotId};
        pathTable{slotId} = path(1);
    end
    
    surplusCapacityTable = zeros(size(pathTable)) + model.maxCapacityOfDrone * 1;   % 剩余容量表
    surplusCapacityTable(:, 1) = model.maxDistanceOfTruck;
    
    customerIds = randperm(model.numOfCustomer);

    % randTruck = 0.8;
    for i = 1: model.numOfCustomer
        cId = customerIds(i);
        cDemand = model.demandOfCustomer(cId);
        slotIds = find(surplusCapacityTable >= cDemand);                        % 可放得下的候选槽
        slotIdsOfTruck = slotIds(slotIds <= model.numOfTruck);                  % 卡车槽
        slotIdsOfDrone = slotIds(slotIds > model.numOfTruck);                   % 无人机槽
        
        candidateSlotIds = slotIds;
        if rand() < randTruck
            candidateSlotIds = slotIdsOfTruck;
            if isempty(candidateSlotIds)
                candidateSlotIds = slotIdsOfDrone;
            end
        else
            candidateSlotIds = slotIdsOfDrone;
            if isempty(candidateSlotIds)
                candidateSlotIds = slotIdsOfTruck;
            end
        end

        goalSlotId = candidateSlotIds(randi([1 length(candidateSlotIds)]));
        pathTable{goalSlotId} = [pathTable{goalSlotId} cId];
        surplusCapacityTable(goalSlotId) = surplusCapacityTable(goalSlotId) - cDemand;
    end
    for slotId = 1: numel(pathTable)
        path = pathTable{slotId};
        pathTable{slotId} = [path path(1)];
    end
end

function [individual] = getIndividualByGreedy(greedyEpsilon, probMatOfTruck, probMatOfDrone, randTruck, model)
    [pathTable] = allocateCustomer(randTruck, model);                       % 随机分配客户
    for truckId = 1: size(pathTable, 1)
        truckPath = pathTable{truckId, 1};
        if length(truckPath) > 3
            pointSet = truckPath(1: end - 1);
            startPoint = pointSet(1);
            newTruckPath = getPathByGreedy(startPoint, pointSet, probMatOfTruck, greedyEpsilon);
            newTruckPath = [newTruckPath truckPath(end)];
            pathTable{truckId, 1} = newTruckPath;
        end
        
        for j = 2: size(pathTable, 2)
            dronePath = pathTable{truckId, j};
            if length(dronePath) > 4
                pointSet = dronePath(2: end - 1);
                startPoint = pointSet(1);
                newDronePath = getPathByGreedy(startPoint, pointSet, probMatOfDrone, greedyEpsilon);
                newDronePath = [dronePath(1) newDronePath dronePath(end)];
                pathTable{truckId, j} = newDronePath;
            end
        end
    end
    [individual] = model.getPathTableToIndividual(pathTable, model);
end

% 调整卡车无人机具体路径
function [path] = getPathByGreedy(startPoint, pointSet, probMat, greedyEpsilon)
    numOfDecVariables = length(pointSet);
    path = zeros(1, numOfDecVariables);
    path(1) = startPoint;
    visiting = pointSet;
    
    index = find(pointSet == startPoint, 1);
    visiting(index) = [];                                                   % 待访问的点
    
    for i = 2 : numOfDecVariables
        point = path(i - 1);                                                % 当前点
        proOfVisiting = probMat(point, visiting);                           % 待访问点的选择概率分布
        if rand() < greedyEpsilon
            [~, selectI] = max(proOfVisiting);
        else
            proOfVisiting = proOfVisiting / sum(proOfVisiting);
            proCum = cumsum(proOfVisiting);
            selectI = find(proCum >= rand(), 1);
            if isempty(selectI)
                selectI = randperm(length(visiting), 1);
            end
        end
        visited = visiting(selectI);                                        % 选中的点
        path(i) = visited;
        visiting(selectI) = [];
    end
end









