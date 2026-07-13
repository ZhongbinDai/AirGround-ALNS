function [model] = initModel4(problemId, numOfDroneMaxDepart)
% 策略2选1
    fileName = sprintf('./data/%d.txt', problemId);                         % 数据集
    [data1, data2] = readTruckDroneTxt(fileName);                           % 读取TXT数据集

    model.numOfCustomer = size(data1, 1) - 1;                               % 客户数量
    model.coordOfCustomer = data1(1: model.numOfCustomer, 1: 2);            % 客户点坐标
    model.demandOfCustomer = data1(1: model.numOfCustomer, 3);              % 客户点需求
    model.coordOfCentre = data1(end, 1: 2);	                                % 供应点坐标
    model.numOfCentre = size(model.coordOfCentre, 1);                       % 供应点数量

    model.numOfDrone = data2(1);                                            % 每辆卡车的无人机数量
    model.numOfTruck = data2(2);                                            % 卡车数量
    model.maxCapacityOfTruck = data2(3);                                    % 卡车最大容量
    model.maxCapacityOfDrone = data2(4);                                    % 无人机最大容量
    model.maxDistanceOfTruck = data2(5);                                    % 卡车最大行驶距离
    model.maxDistanceOfDrone = data2(6);                                    % 无人机最大行驶距离
    model.speedOfTruck = data2(7);                                          % 卡车速度
    model.speedOfDrone = data2(8);                                          % 无人机速度
    model.problemId = problemId;                                            % 问题id

    model.penaltyFactor = 10 ^ 8;                                           % 惩罚因子
    model.weightOfObjs = [1e-7 1e-7 1];                                     % 目标权重
    % model.weightOfObjs = [1 1 0];
    
    model.numOfDronePotentialCustomer = sum(model.demandOfCustomer <= model.maxCapacityOfDrone);        % 需求量小于无人机容量的客户数目
    model.numOfDroneMaxDepart = getNumOfDroneMaxDepart(numOfDroneMaxDepart, model);                     % 每架无人机最大出发次数
    [model.distanceMatOfTruck, model.coord] = getDistanceMat(1, model);     % 卡车的距离矩阵,L1范数,曼哈顿距离
    [model.distanceMatOfDrone] = getDistanceMat(2, model);                  % 无人机的距离矩阵,L2范数,欧氏距离
    model.timeMatOfTruck = model.distanceMatOfTruck / model.speedOfTruck;
    model.timeMatOfDrone = model.distanceMatOfDrone/ model.speedOfDrone;
    model.sequence = getSequence(model);                                    % 决策变量序列
    model.numOfDecVariables = length(model.sequence);                       % 决策变量维度

    model.initIndividual = @initIndividual;                                 % 初始化个体
    model.repairIndividual = @repairIndividual;                             % 修复个体
    model.getIndividualFitness = @getIndividualFitness;                     % 计算个体适应度
    model.printIndividual = @printIndividual;                               % 打印结果
    model.showIndividual = @showIndividual;                                 % 个体可视化

    model.getPathTable = @getPathTable;
    model.getTruckAndDronePathTable = @getTruckAndDronePathTable;
    model.getPathLengthTable = @getPathLengthTable;
    model.getRealDronePathTableCell = @getRealDronePathTableCell;
    model.getTruckPathDistanceTable = @getTruckPathDistanceTable;
    model.getDronePathDistanceTableCell = @getDronePathDistanceTableCell;
    model.getAllCost = @getAllCost;
    model.printIndividualOfCompetition = @printIndividualOfCompetition;
    model.printSubmitResult = @printSubmitResult;
    model.repairIndividual1 = @repairIndividual1;
    model.repairIndividual2 = @repairIndividual2;
    model.saveResult = @saveResult;
    model.getPathTableToIndividual = @getPathTableToIndividual;
    model.drawPartPath = @drawPartPath;
    model.getCost3 = @getCost3;
    model.drawPoints = @drawPoints;
    model.drawPath2 = @drawPath2;
    model.getOverDistanceOfDrone = @getOverDistanceOfDrone;
    model.getNumOfSamePoint = @getNumOfSamePoint;
    model.printSubmitResult2 = @printSubmitResult2;
    model.showIndividual2 = @showIndividual2;
    model.truckDronePathTableToIndividual = @truckDronePathTableToIndividual;
    model.printIndividual2 = @printIndividual2;
end

%% 自适应计算每架无人机最大出发次数
function [numOfDroneMaxDepart] = getNumOfDroneMaxDepart(numOfDroneMaxDepart, model)
    numOfDronePotentialCustomer = model.numOfDronePotentialCustomer;                                % 需求量小于无人机容量的客户数目
    maxNumDepart = ceil(numOfDronePotentialCustomer / model.numOfTruck / model.numOfDrone);         % 最大出发次数
    if isempty(numOfDroneMaxDepart)
        numOfDroneMaxDepart = maxNumDepart;
    else
        if maxNumDepart < numOfDroneMaxDepart
            numOfDroneMaxDepart = maxNumDepart;                                                     % 是否设这个上限,待商榷
        end
    end
end

% 计算任意两点距离矩阵
function [distanceMat, coord] = getDistanceMat(p, model)
    coordOfCentre = repmat(model.coordOfCentre, [model.numOfTruck, 1]);
    coord = [model.coordOfCustomer; coordOfCentre];                         % 前numOfCustomer个是用户坐标，剩余的为供应点坐标
    num = size(coord, 1);                                                   % 坐标数量
    distanceMat = zeros(num, num);
    for i = 1 : num
        coordI = coord(i, :);
        for j = 1 : num
            coordJ = coord(j, :);
            distanceMat(i, j) = norm(coordI - coordJ, p);
        end
    end
end

% 决策变量序列,所有客户点+货车id+每架无人机每次出发id
function [sequence] = getSequence(model)
    numOfCustomer = model.numOfCustomer;                                    % 客户数量
    numOfTruck = model.numOfTruck;                                          % 卡车数量
    numOfDrone = model.numOfDrone;                                          % 每辆卡车的无人机数量
    numOfDroneMaxDepart = model.numOfDroneMaxDepart;                        % 无人机最大出发次数
    temp = repmat((1: numOfTruck)', [1, 1 + numOfDrone * numOfDroneMaxDepart]) + numOfCustomer;
    temp = sort(reshape(temp, [1, numel(temp)]));
    sequence = [1: numOfCustomer, temp];
end

%% 随机初始化个体
function [individual] = initIndividual(model)
    [sequence] = model.sequence;
    individual = sequence(randperm(length(sequence)));
    [individual] = repairIndividual(individual, model);
end

% 修复个体
function [newIndividual] = repairIndividual(individual, model)
    [newIndividual] = repairIndividual1(individual, model);
    [newIndividual] = repairIndividual2(newIndividual, model);
end

% 路径第一个点必须为起点
function [newIndividual] = repairIndividual1(individual, model)
    numOfCustomer = model.numOfCustomer;                                    % 客户数量
    newIndividual = individual;
    centreIndex = find(individual > numOfCustomer, 1);
    newIndividual([1 centreIndex]) = newIndividual([centreIndex 1]);
end

% 无人机超载修复
function [newIndividual] = repairIndividual2(individual, model)
    [pathTable] = getPathTable(individual, model);
    newPathTable = pathTable;
    for truckId = 1: size(pathTable, 1)
        truckPath = pathTable{truckId, 1};
        tCustomerIds = truckPath(2: end - 1);                               % 卡车的客户id
        for i = 2: size(pathTable, 2)
            dronePath = pathTable{truckId, i};
            if length(dronePath) > 2
                [removeCustomerIds, reserveCustomerIds, newDronePath] = getRemoveDroneCustomerIds(dronePath, model);
                tCustomerIds = [tCustomerIds removeCustomerIds];
                newPathTable{truckId, i} = newDronePath;
            end
        end
        newTruckPath = [truckPath(1) tCustomerIds truckPath(end)];
        newPathTable{truckId, 1} = newTruckPath;
    end
    newIndividual = getPathTableToIndividual(newPathTable, model);
end

% 提出无人机客户中超出无人机最大容量的客户分到货车末尾，保证无人机不超重
function [removeCustomerIds, reserveCustomerIds, newDronePath] = getRemoveDroneCustomerIds(dronePath, model)
    dCustomerIds = dronePath(2: end - 1);                                   % 无人机的客户id
    removeFlagArray = zeros(size(dCustomerIds));                            % 0保留,1移除
    sumLoad = 0;
    for i = 1: length(dCustomerIds)
        cId = dCustomerIds(i);
        if sumLoad + model.demandOfCustomer(cId) > model.maxCapacityOfDrone
            removeFlagArray(i) = 1;
        else
            sumLoad = sumLoad + model.demandOfCustomer(cId);
        end
    end
    removeIndex = find(removeFlagArray == 1);
    removeCustomerIds = dCustomerIds(removeIndex);                          % 超出无人机重量的客户id
    reserveCustomerIds = dCustomerIds;                                      % 保留的无人机的客户id
    reserveCustomerIds(removeIndex) = [];
    newDronePath = [dronePath(1) reserveCustomerIds dronePath(end)];        % 超出无人机重量的客户放在货车末尾
end

% pathTable转成决策变量(个体)
function [individual] = getPathTableToIndividual(pathTable, model)
    individualTemp = zeros(1, model.numOfDecVariables);
    k = 0;
    for i = 1: numel(pathTable)
        path = pathTable{i};
        n = max(length(path) - 1, 0);
        individualTemp(k + 1: k + n) = path(1: end - 1);
        k = k + n;
    end
    individual = individualTemp(1: k);
end

% 路径表中每个路径的长度
function [pathLengthTable] = getPathLengthTable(pathTable)
    pathLengthTable = zeros(size(pathTable));
    [numOfTruck, numOfDepart] = size(pathTable);
    for i = 1: numOfTruck
        for j = 1: numOfDepart
            path = pathTable{i, j};
            pathLengthTable(i, j) = length(path);
        end
    end
end

% 空路线后移
function [newPathTable] = emptyPathBackwardMove(pathTable)
    [pathLengthTable] = getPathLengthTable(pathTable);
    newPathTable = pathTable;
    for i = 1: size(pathTable, 1)
        Index1 = find(pathLengthTable(i, :) > 2);
        Index2 = find(pathLengthTable(i, :) <= 2);
        k = 0;
        for j = 1: length(Index1)
            k = k + 1;
            path = pathTable{i, Index1(j)};
            newPathTable{i, k} = path;
        end
        for j = 1: length(Index2)
            k = k + 1;
            path = pathTable{i, Index2(j)};
            newPathTable{i, k} = path;
        end
    end
end

%% 每辆车的路线+无人机路线
function [pathTable] = getPathTable(individual, model)
    numOfCustomer = model.numOfCustomer;                                    % 客户数量
    numOfTruck = model.numOfTruck;                                          % 卡车数量
    numOfDrone = model.numOfDrone;                                          % 每辆卡车的无人机数量
    numOfDroneMaxDepart = model.numOfDroneMaxDepart;                        % 无人机最大出发次数

    pathTable = cell(numOfTruck, 1 + numOfDrone * numOfDroneMaxDepart);     % 每辆车的路线+无人机路线
    
    pathNumTable = zeros(numOfTruck, 1);
    centreIndex = find(individual > numOfCustomer);                         % 查询车辆所在序列位置
    centreIndexEnd = [centreIndex(2 : end) - 1 , length(individual)];
    lengthOfPathArray = centreIndexEnd - centreIndex;                       % 每条路线的长度
    startPointOfVehicleArray = individual(centreIndex);                     % 每辆车的起点

    for i = 1 : length(lengthOfPathArray)
        k = startPointOfVehicleArray(i) - numOfCustomer;                    % 起点信息
        pathNumTable(k) = pathNumTable(k) + 1;
        startI = centreIndex(i);
        endI = centreIndexEnd(i);
        pathOfVehicle = [individual(startI : endI) individual(startI)];
        pathTable{k, pathNumTable(k)} = pathOfVehicle;
    end
end

% 分离实际的卡车路线、计划的无人机路线
function [truckPathTable, planDronePathTableCell] = getTruckAndDronePathTable(pathTable, model)
    numOfTruck = model.numOfTruck;                                          % 卡车数量
    numOfDrone = model.numOfDrone;                                          % 每辆卡车的无人机数量
    numOfDroneMaxDepart = model.numOfDroneMaxDepart;                        % 无人机最大出发次数
    truckPathTable = pathTable(:, 1);
    dronePathTable = pathTable(:, 2: end);
    planDronePathTableCell = cell(numOfTruck, 1);
    for truckId = 1: numOfTruck
        planDronePathTable = reshape(dronePathTable(truckId, :), [numOfDrone, numOfDroneMaxDepart]);              % 竖放
        % planDronePathTable = reshape(dronePathTable(truckId, :), [numOfDroneMaxDepart, numOfDrone])';           % 横放
        planDronePathTableCell{truckId} = emptyPathBackwardMove(planDronePathTable);                              % 无人机路径每行,空路线后移
    end
end


%% 每辆车的行驶距离、发车状态、负载量
function [truckPathDistanceTable, truckStateTable, truckLoadTable] = getTruckPathDistanceTable(truckPathTable, model)
    truckPathDistanceTable = zeros(size(truckPathTable));                   % 每辆车的行驶距离
    truckStateTable = zeros(size(truckPathTable));                          % 每辆车的发车状态
    truckLoadTable = zeros(size(truckPathTable));                           % 每辆车的负载量
    
    for truckId = 1: numel(truckPathTable)
        path = truckPathTable{truckId};
        truckLoadTable(truckId) = sum(model.demandOfCustomer(path(2: end - 1)));
        truckPathDistanceTable(truckId) = getPathDistance(path, model.distanceMatOfTruck);
        truckStateTable(truckId) = length(path) > 2;
    end
end

% 每辆车每个无人机的行驶距离、发车状态、负载量
function [dronePathDistanceTableCell, droneStateTableCell, droneLoadTableCell] = getDronePathDistanceTableCell(realDronePathTableCell, model)
    dronePathDistanceTableCell = cell(size(realDronePathTableCell));
    droneStateTableCell = cell(size(realDronePathTableCell));
    droneLoadTableCell = cell(size(realDronePathTableCell));

    for truckId = 1: numel(realDronePathTableCell)
        realDronePathTable = realDronePathTableCell{truckId};
        [dronePathDistanceTable, droneStateTable, droneLoadTable] = getDronePathDistanceTable(realDronePathTable, model);
        dronePathDistanceTableCell{truckId} = dronePathDistanceTable;
        droneStateTableCell{truckId} = droneStateTable;
        droneLoadTableCell{truckId} = droneLoadTable;
    end
end

% 每个无人机的行驶距离、发车状态、负载量
function [dronePathDistanceTable, droneStateTable, droneLoadTable] = getDronePathDistanceTable(realDronePathTable, model)
    dronePathDistanceTable = zeros(size(realDronePathTable));               % 每辆车的行驶距离
    droneStateTable = zeros(size(realDronePathTable));                      % 每辆车的发车状态
    droneLoadTable = zeros(size(realDronePathTable));                       % 每辆车的负载量
    
    for i = 1: numel(realDronePathTable)
        path = realDronePathTable{i};
        if ~isempty(path)
            droneLoadTable(i) = sum(model.demandOfCustomer(path(2: end - 1)));
            dronePathDistanceTable(i) = getPathDistance(path, model.distanceMatOfDrone);
            droneStateTable(i) = length(path) > 2;
        end
    end
end

% 计算路线距离
function [pathDistance] = getPathDistance(path, distanceMat)
    pathDistance = 0;  
    for i = 1: length(path) - 1
        p1 = path(i);
        p2 = path(i + 1);
        dis = distanceMat(p1, p2);                                          % 两点间的距离
        pathDistance = pathDistance + dis;                                  % 总路程，越小越好
    end
end



%% 实际的无人机路线 无人机不能从仓库到仓库
function [realDronePathTableCell] = getRealDronePathTableCell(truckPathTable, planDronePathTableCell, model)
    realDronePathTableCell = cell(size(planDronePathTableCell));
    startTime = 0;
    for truckId = 1: numel(realDronePathTableCell)
        truckPath = truckPathTable{truckId};
        planDronePathTable = planDronePathTableCell{truckId};
        realDronePathTable = cell(size(planDronePathTable));
        for droneId = 1: size(planDronePathTable, 1)
            planDronePathOfOne = planDronePathTable(droneId, :);
            % realDronePathTable(droneId, :) = getRealDronePathCell1(planDronePathOfOne, truckPath, model);
            % 2中策略选其1
            realDronePathTable(droneId, :) = getBestDronePathCell(planDronePathOfOne, truckPath, model);
        end
        realDronePathTableCell{truckId} = realDronePathTable;
    end
end

% 3中策略选其1
function [realDronePathCell] = getBestDronePathCell(planDronePathOfOne, truckPath, model)
    % realDronePathCell0 = getRealDronePathCell0(planDronePathOfOne, truckPath, model);
    realDronePathCell1 = getRealDronePathCell1(planDronePathOfOne, truckPath, model);
    realDronePathCell2 = getRealDronePathCell2(planDronePathOfOne, truckPath, model);

    % [completionTime0] = getMaxFinishTime({truckPath}, {realDronePathCell0}, model);
    [completionTime1] = getMaxFinishTime({truckPath}, {realDronePathCell1}, model);
    [completionTime2] = getMaxFinishTime({truckPath}, {realDronePathCell2}, model);

    
    % drawRealDronePathCell(truckPath, realDronePathCell1, model);
    % drawRealDronePathCell(truckPath, realDronePathCell2, model);
    % drawRealDronePathCell(truckPath, realDronePathCell0, model);
    
    
    % [dronePathDistanceTable0, ~, ~] = getDronePathDistanceTable(realDronePathCell0, model);
    [dronePathDistanceTable1, ~, ~] = getDronePathDistanceTable(realDronePathCell1, model);
    [dronePathDistanceTable2, ~, ~] = getDronePathDistanceTable(realDronePathCell2, model);
    
    
    maxDistanceOfDrone = model.maxDistanceOfDrone;
    % overDistance0 = sum((dronePathDistanceTable0 - maxDistanceOfDrone) .* (dronePathDistanceTable0 > maxDistanceOfDrone), 'all');
    overDistance1 = sum((dronePathDistanceTable1 - maxDistanceOfDrone) .* (dronePathDistanceTable1 > maxDistanceOfDrone), 'all');
    overDistance2 = sum((dronePathDistanceTable2 - maxDistanceOfDrone) .* (dronePathDistanceTable2 > maxDistanceOfDrone), 'all');
    
    % f0 = completionTime0 + model.penaltyFactor * overDistance0;
    f1 = completionTime1 + model.penaltyFactor * overDistance1;
    f2 = completionTime2 + model.penaltyFactor * overDistance2;
    [~, I] = min([f1 f2]);
    if I == 1
        realDronePathCell = realDronePathCell1;
    else
        realDronePathCell = realDronePathCell2;
    end
end

function [newRealDronePathCell] = updateRealDronePathCell(realDronePathCell, truckPath, model)
    newRealDronePathCell = realDronePathCell;
    for departId = length(newRealDronePathCell): -1: 1
        dronePath2 = newRealDronePathCell{departId};
        
        if length(dronePath2) > 2
            partDistance = getPathDistance(dronePath2(2: end), model.distanceMatOfDrone);
            p2 = dronePath2(2);
            if departId > 1
                dronePath1 = newRealDronePathCell{departId - 1};
            else
                dronePath1 = truckPath(1);
            end

            pStart = dronePath1(end);
            pEnd = dronePath2(1);
            indexStart = find(truckPath == pStart, 1);
            endStart = find(truckPath == pEnd, 1);
            for i = indexStart: endStart - 1
                p1 = truckPath(indexStart);
                droneDistance = partDistance + model.distanceMatOfDrone(p1, p2);
                if droneDistance <= model.maxDistanceOfDrone
                    dronePath2(1) = p1;
                    newRealDronePathCell{departId} = dronePath2;
                    break;
                end
            end
        end
    end
end

% 无人机的实际路线,方案1顺序插入
function [realDronePathCell] = getRealDronePathCell1(planDronePathCell, truckPath, model)
    numDroneDepart = 0;                                                     % 无人机的有效出发次数
    for departId = 1: numel(planDronePathCell)
        if length(planDronePathCell{departId}) > 2
            numDroneDepart = numDroneDepart + 1;
        end
    end

    indexStart = 1;
    realDronePathCell = cell(size(planDronePathCell));
    for departId = 1: numel(planDronePathCell)
        planDronePath = planDronePathCell{departId};
        if length(planDronePath) > 2
            indexEnd = length(truckPath) - (numDroneDepart - departId);
            if indexEnd <= indexStart
                indexEnd = min(indexStart + 1, length(truckPath));
            end
            % fprintf('%d %d\n', indexStart, indexEnd);
            truckPathPart = truckPath(indexStart: indexEnd);
            [realDronePath, indexAdd] = getRealDronePath(planDronePath, truckPathPart, model);
            realDronePathCell{departId} = realDronePath;
            indexStart = indexStart + indexAdd;
        end
    end
end

% 无人机的实际路线,方案2距离最短优先插入
function [realDronePathCell] = getRealDronePathCell2(planDronePathCell, truckPath, model)
    numDroneDepart = 0;                                                     % 无人机的有效出发次数
    for departId = 1: numel(planDronePathCell)
        if length(planDronePathCell{departId}) > 2
            numDroneDepart = numDroneDepart + 1;
        end
    end

    indexStart = 1;
    realDronePathCell = cell(size(planDronePathCell));
    planDronePathCellTemp = planDronePathCell;
    for departId = 1: numel(planDronePathCellTemp)
        pId = truckPath(indexStart);
        [pDistanceArray] = getPDistanceArray(pId, planDronePathCellTemp, model);
        [~, minIndex] = min(pDistanceArray);
        planDronePath = planDronePathCellTemp{minIndex};

        if length(planDronePath) > 2
            indexEnd = length(truckPath) - (numDroneDepart - departId);
            if indexEnd <= indexStart
                indexEnd = min(indexStart + 1, length(truckPath));
            end
            % fprintf('%d %d\n', indexStart, indexEnd);
            truckPathPart = truckPath(indexStart: indexEnd);
            [realDronePath, indexAdd] = getRealDronePath(planDronePath, truckPathPart, model);
            realDronePathCell{departId} = realDronePath;
            indexStart = indexStart + indexAdd;
        end
        planDronePathCellTemp{minIndex} = [];
    end
end

function [realDronePath, indexAdd] = getRealDronePath(planDronePath, truckPathPart, model)
% 无人机要比卡车先到汇合点
    realDronePath = planDronePath;
    realDronePath(1) = truckPathPart(1);
    if length(truckPathPart) == 1
        realDronePath(end) = truckPathPart(1);
        indexAdd = 0;
        return;
    end

    % 无人机飞行时间(缺最后一节)
    droneFlightTime0 = getPathDistance(realDronePath(1: end - 1), model.distanceMatOfDrone) / model.speedOfDrone;
    truckDriveTime = 0;                                                     % 卡车行驶时间
    for i = 2: length(truckPathPart)
        p1 = truckPathPart(i - 1);
        p2 = truckPathPart(i);
        truckDriveTime = truckDriveTime + model.distanceMatOfTruck(p1, p2) / model.speedOfTruck;
        p3 = planDronePath(end - 1);
        droneFlightTime = droneFlightTime0 + model.distanceMatOfDrone(p3, p2) / model.speedOfDrone;
        % fprintf("%d 车%.2f 机%.2f\n", i, truckDriveTime, droneFlightTime);

        % 或去离无人机最近的点汇合？
        if truckDriveTime > droneFlightTime                                 % 卡车要更慢到达,必须优化,存在优化空间todo
            break;
        end
    end
    realDronePath(end) = truckPathPart(i);
    indexAdd = i - 1;
end

function [realDronePath, indexAdd] = getRealDronePath2(planDronePath, truckPathPart, model)
% 无人机要比卡车先到汇合点
    realDronePath = planDronePath;
    realDronePath(1) = truckPathPart(1);
    if length(truckPathPart) == 1
        realDronePath(end) = truckPathPart(1);
        indexAdd = 0;
        return;
    end

    % 无人机飞行时间(缺最后一节)
    droneFlightTime0 = getPathDistance(realDronePath(1: end - 1), model.distanceMatOfDrone) / model.speedOfDrone;
    truckDriveTime = 0;                                                     % 卡车行驶时间
    for i = 2: length(truckPathPart)
        p1 = truckPathPart(i - 1);
        p2 = truckPathPart(i);
        truckDriveTime = truckDriveTime + model.distanceMatOfTruck(p1, p2) / model.speedOfTruck;
        p3 = planDronePath(end - 1);
        droneFlightTime = droneFlightTime0 + model.distanceMatOfDrone(p3, p2) / model.speedOfDrone;
        % fprintf("%d 车%.2f 机%.2f\n", i, truckDriveTime, droneFlightTime);

        rateSpeed = truckDriveTime / droneFlightTime;
        % 或去离无人机最近的点汇合？
        if rateSpeed > 0.9 && rateSpeed < 1.1
            break;
        end
    end
    realDronePath(end) = truckPathPart(i);
    indexAdd = i - 1;
end

% 点pId到每次无人机的第一个客户的距离矩阵
function [pDistanceArray] = getPDistanceArray(pId, planDronePathCell, model)
    pDistanceArray = inf(size(planDronePathCell));
    for departId = 1: numel(planDronePathCell)
        planDronePath = planDronePathCell{departId};
        if length(planDronePath) > 2
            p1 = planDronePath(2);
            pDistanceArray(departId) = model.distanceMatOfDrone(pId, p1);
        end
    end
end

%% 无人机的实际路线,方案0无人机两头最短路径插入
function [realDronePathCell] = getRealDronePathCell0(planDronePathCell, truckPath, model)
    [pathLengthCell] = getPathLengthTable(planDronePathCell);
    validN = sum(pathLengthCell > 2);
    
    [customerPairArray] = getCustomerPairArray(truckPath);                  % 路线对
    if size(customerPairArray, 1) < validN
        customerPairArray = [customerPairArray; zeros(validN - size(customerPairArray, 1), 2) + truckPath(1)];
    end

    visitedArray = zeros(size(customerPairArray, 1), 1);
    dronePathIdArrayOfPair = zeros(length(visitedArray), 1);
    for i = 1: length(planDronePathCell)
        planDronePath = planDronePathCell{i};
        if length(planDronePath) > 2
            distanceArray = inf(length(visitedArray), 1);
            customerId1 = planDronePath(2);
            customerId2 = planDronePath(end - 1);
            for j = 1: length(visitedArray)
                if visitedArray(j) == 0
                    p1 = customerPairArray(j, 1);
                    p2 = customerPairArray(j, 2);
                    
                    distanceArray(j) = model.distanceMatOfDrone(p1, customerId1) + model.distanceMatOfDrone(customerId2, p2);
                end
            end
            [~, minI] = min(distanceArray);
            dronePathIdArrayOfPair(minI) = i;
            visitedArray(minI) = 1;
        end
    end

    [realDronePathCell] = updateDronePathCell(customerPairArray, dronePathIdArrayOfPair, planDronePathCell, model);
    [realDronePathCell] = updateRealDronePathCell(realDronePathCell, truckPath, model); 
end

% 可优化
function [newDronePathCell] = updateDronePathCell(customerPairArray, dronePathIdArrayOfPair, planDronePathCell, model)
    goalI = find(dronePathIdArrayOfPair == 0);
    for i = 1: length(goalI)
        I = goalI(i);
        if dronePathIdArrayOfPair(I) == 0
            Is = find(dronePathIdArrayOfPair > 0);
            p1 = customerPairArray(I, 1);
            p2 = customerPairArray(I, 2);
            temps = find(Is < I);
            if isempty(temps)
                IL = [];
            else
                IL = Is(temps(end));
            end
            IR = Is(find(Is > I, 1));
            gainL = [];
            gainR = [];
            if ~isempty(IL)
                realDronePath = planDronePathCell{dronePathIdArrayOfPair(IL)};
                customerIdL = realDronePath(end - 1);
                pL = customerPairArray(IL, 2);
                gainL = model.distanceMatOfDrone(customerIdL, pL) - model.distanceMatOfDrone(customerIdL, p2);
            end
            if ~isempty(IR)
                realDronePath = planDronePathCell{dronePathIdArrayOfPair(IR)};
                customerIdR = realDronePath(2);
                pR = customerPairArray(IR, 1);
                gainR = model.distanceMatOfDrone(pR, customerIdR) - model.distanceMatOfDrone(p1, customerIdR);
            end
            if ~isempty(gainL)
                if gainL > 0
                    if isempty(gainR) || gainL > gainR
                        dronePathIdArrayOfPair(IL: I) = dronePathIdArrayOfPair(IL);
                    end
                end
            end

            if ~isempty(gainR)
                if gainR > 0
                    if isempty(gainL) || gainL < gainR
                        dronePathIdArrayOfPair(I: IR) = dronePathIdArrayOfPair(IR);
                    end
                end
            end
        end
    end

    newDronePathCell = cell(size(planDronePathCell));
    counter = 0;
    i = 1;
    while i <= length(dronePathIdArrayOfPair)
        pId = i;
        if dronePathIdArrayOfPair(pId) ~= 0
            j = i + 1;
            % 出现了严重错误*，已修复
            while j <= length(dronePathIdArrayOfPair) && dronePathIdArrayOfPair(pId) == dronePathIdArrayOfPair(j)
                j = j + 1;
            end
            j = j - 1;
            
            p1 = customerPairArray(i, 1);
            p2 = customerPairArray(j, 2);
            realDronePath = planDronePathCell{dronePathIdArrayOfPair(pId)};
            counter = counter + 1;
            newDronePathCell{counter} = [p1 realDronePath(2: end - 1) p2];
            if counter == length(planDronePathCell)
                break;
            end
            i = j;
        end
        i = i + 1;
    end
end

% 拆解路线对
function [customerPairArray] = getCustomerPairArray(truckPath)
    if length(truckPath) > 2
        customerPairArray = zeros(length(truckPath) - 1, 2);
        for i = 1: length(truckPath) - 1
            if truckPath(i) ~= truckPath(i + 1)
                customerPairArray(i, :) = [truckPath(i) truckPath(i + 1)];
            end
        end
    else
        customerPairArray = [];
    end
end

%% 计算该车辆、无人机的最大完成时间 [是否正确存疑]
function [completionTime, truckDroneTimeResult] = getTruckCompletionTime(truckPath, realDronePathTable, startTime, model)
    droneDepartPointIndexsArray = cell(model.numOfDrone, 1);
    droneFallPointIndexsArray = cell(model.numOfDrone, 1);
    for droneId = 1: size(realDronePathTable, 1)
        realDronePathArray = realDronePathTable(droneId, :);
        % 无人机每次飞行的起飞点、降落点，对应在车辆路线上的索引
        [droneDepartPointIndexs, droneFallPointIndexs] = getDroneDepartFallPointIndexs(realDronePathArray, truckPath);
        droneDepartPointIndexsArray{droneId} = droneDepartPointIndexs;
        droneFallPointIndexsArray{droneId} = droneFallPointIndexs;
    end
    [realDronePathDistanceTable, ~, ~] = getDronePathDistanceTable(realDronePathTable, model);
    realDronePathFlightTimeTable = realDronePathDistanceTable / model.speedOfDrone;         % 无人机每次实际飞行时间

    truckArrivalTimeArray = zeros(size(truckPath));                                         % 卡车每个点的到达时间
    truckArrivalTimeArray(1) = startTime;
    truckLeaveTimeArray = zeros(size(truckPath));                                           % 卡车每个点的离开时间
    truckLeaveTimeArray(1) = startTime;

    realDroneArrivalTimeTable = zeros(size(realDronePathTable)) - 1;                        % 无人机每次飞行的到达时刻
    realDroneLeaveTimeTable = zeros(size(realDronePathTable)) - 1;                          % 无人机每次飞行的起飞时刻
    realDroneFinishTimeTable = zeros(size(realDronePathTable)) - 1;                         % 无人机每次飞行的完成时刻
    realDroneWaitingTimeTable = zeros(size(realDronePathTable)) - 1;                        % 无人机每次飞行的等待时长

    droneCurrentTimeArray = zeros(size(realDronePathTable, 1), 1) + startTime;              % 每辆无人机的当前时刻
    droneFlightNumArray = zeros(size(realDronePathTable, 1), 1);                            % 每辆无人机飞行次数计数
    for i = 2: length(truckPath)
        p1 = truckPath(i - 1);
        p2 = truckPath(i);                                                                  % 当前卡车访问的客户
        truckDriveTime = model.distanceMatOfTruck(p1, p2) / model.speedOfTruck;             % 卡车行驶时间
        truckArrivalTimeArray(i) = truckLeaveTimeArray(i - 1) + truckDriveTime;             % 卡车到达时刻
        truckLeaveTimeArray(i) = truckArrivalTimeArray(i);                                  % 卡车离开时刻
        
        for droneId = 1: size(realDronePathTable, 1)
            droneDepartPointIndexs = droneDepartPointIndexsArray{droneId};
            droneFallPointIndexs = droneFallPointIndexsArray{droneId};
            departId = find(droneFallPointIndexs == i, 1);                                  % 无人机第departId次飞行在当前客户点停靠
            if ~isempty(departId)                                                           % 无人机飞行路径
                droneDepartPointIndex = droneDepartPointIndexs(departId);                   % 无人机此次飞行的起点Index(相对于卡车路线)
                droneCurrentTime = droneCurrentTimeArray(droneId);                          % 无人机的当前时刻
                truckArrivalTime = truckArrivalTimeArray(droneDepartPointIndex);            % 卡车到达无人机起点时刻
                droneStartTime = max(truckArrivalTime, droneCurrentTime);                   % 无人机实际起飞时刻
                dronePathFlightTime = realDronePathFlightTimeTable(droneId, departId);      % 无人机飞行时间
                droneCurrentTimeArray(droneId) = droneStartTime + dronePathFlightTime;

                realDroneArrivalTimeTable(droneId, departId) = droneCurrentTime;                        % 无人机每次飞行的到达时刻
                realDroneLeaveTimeTable(droneId, departId) = droneStartTime;                            % 无人机每次飞行的起飞时刻
                realDroneFinishTimeTable(droneId, departId) = droneStartTime + dronePathFlightTime;     % 无人机每次飞行的完成时刻
                realDroneWaitingTimeTable(droneId, departId) = droneStartTime - droneCurrentTime;       % 无人机每次飞行的等待时长
                
                if droneCurrentTimeArray(droneId) > truckLeaveTimeArray(i)                              % 如果在汇合点,无人机先到达
                    truckLeaveTimeArray(i) = droneCurrentTimeArray(droneId);                            % 卡车离开时刻更新
                end
                droneFlightNumArray(droneId) = droneFlightNumArray(droneId) + 1;
            end
        end
    end

    % 无人机终点-终点情况 [后来官方舍弃了]
    for droneId = 1: size(realDronePathTable, 1)
        droneFlightNum = droneFlightNumArray(droneId);
        droneFallPointIndexs = droneFallPointIndexsArray{droneId};
        if droneFlightNum < length(droneFallPointIndexs)
            truckArrivalTime = truckArrivalTimeArray(end);                  % 卡车到达无人机起点时刻
            droneCurrentTime = droneCurrentTimeArray(droneId);              % 无人机的当前时刻
            droneStartTime = max(truckArrivalTime, droneCurrentTime);       % 无人机实际起飞时刻
            droneCurrentTimeArray(droneId) = droneStartTime;
            for departId = droneFlightNum + 1: length(droneFallPointIndexs)
                dronePathFlightTime = realDronePathFlightTimeTable(droneId, departId);      % 无人机飞行时间
                droneArrivalTime = droneCurrentTimeArray(droneId);                          % 无人机起飞时刻
                droneFinishTime = droneArrivalTime + dronePathFlightTime;                   % 无人机完成时刻
                droneCurrentTimeArray(droneId) = droneFinishTime;

                realDroneArrivalTimeTable(droneId, departId) = droneArrivalTime;            % 无人机每次飞行的到达时刻
                realDroneLeaveTimeTable(droneId, departId) = droneArrivalTime;              % 无人机每次飞行的起飞时刻
                realDroneFinishTimeTable(droneId, departId) = droneFinishTime;              % 无人机每次飞行的完成时刻
                realDroneWaitingTimeTable(droneId, departId) = 0;                           % 无人机每次飞行的等待时长
            end
        end
    end
    completionTime = max([truckLeaveTimeArray(end); droneCurrentTimeArray]);                % 实际完成时间
    truckLeaveTimeArray(end) = completionTime;

    truckDroneTimeResult.truckArrivalTimeArray = truckArrivalTimeArray;
    truckDroneTimeResult.truckLeaveTimeArray = truckLeaveTimeArray;
    truckDroneTimeResult.realDroneArrivalTimeTable = realDroneArrivalTimeTable;
    truckDroneTimeResult.realDroneLeaveTimeTable = realDroneLeaveTimeTable;
    truckDroneTimeResult.realDroneFinishTimeTable = realDroneFinishTimeTable;
    truckDroneTimeResult.realDroneWaitingTimeTable = realDroneWaitingTimeTable;
end

% 无人机每次飞行的起飞点、降落点
function [droneDepartPointIds, droneFallPointIds] = getDroneDepartFallPointIds(dronePathArray)
    k = 0;
    droneDepartPointIds = zeros(1, length(dronePathArray));                 % 无人机每次飞行的起飞点
    droneFallPointIds = zeros(1, length(dronePathArray));                   % 无人机每次飞行的降落点（和卡车汇合）
    for i = 1: length(dronePathArray)
        dronePath = dronePathArray{i};
        if length(dronePath) > 2
            droneDepartPointIds(i) = dronePath(1);
            droneFallPointIds(i) = dronePath(end);
            k = k + 1;
        else
            break;
        end
    end
    droneDepartPointIds = droneDepartPointIds(1: k);
    droneFallPointIds = droneFallPointIds(1: k);
end

% 无人机每次飞行的起飞点、降落点,对应在车辆路线上的索引
function [droneDepartPointIndexs, droneFallPointIndexs] = getDroneDepartFallPointIndexs(dronePathArray, truckPath)
    [droneDepartPointIds, droneFallPointIds] = getDroneDepartFallPointIds(dronePathArray);
    droneDepartPointIndexs = zeros(size(droneDepartPointIds));
    droneFallPointIndexs = zeros(size(droneFallPointIds));
    for i = 1: length(droneDepartPointIndexs)
        droneDepartPointIndexs(i) = find(truckPath == droneDepartPointIds(i), 1);
        droneFallPointIndexs(i) = find(truckPath == droneFallPointIds(i), 1, 'last');
        if i > 1
            if droneDepartPointIndexs(i) < droneFallPointIndexs(i - 1)
                droneDepartPointIndexs(i) = droneFallPointIndexs(i);
            end
        end
    end
end

% 完成全部任务时刻
function [truckDroneTimeResultCell] = getTruckDroneTimeResultCell(truckPathTable, realDronePathTableCell, model)
    numOfTruck = model.numOfTruck;
    completionTimeArray = zeros(numOfTruck, 1);
    truckDroneTimeResultCell = cell(numOfTruck, 1);
    startTime = 0;
    for truckId = 1: numOfTruck
        truckPath = truckPathTable{truckId};
        realDronePathTable = realDronePathTableCell{truckId};
        [completionTime, truckDroneTimeResult] = getTruckCompletionTime(truckPath, realDronePathTable, startTime, model);
        truckDroneTimeResultCell{truckId} = truckDroneTimeResult;
        completionTimeArray(truckId) = completionTime;
    end
end

%% 卡车容量约束
function [overloadOfTruck] = getOverloadOfTruck(truckLoadTable, droneLoadTableCell, model)
    maxCapacityTable = zeros(size(truckLoadTable)) + model.maxCapacityOfTruck;
    
    totalTruckLoadTable = zeros(size(truckLoadTable));
    for i = 1: numel(totalTruckLoadTable)
        droneLoadTable = droneLoadTableCell{i};
        totalTruckLoadTable(i) = truckLoadTable(i) + sum(droneLoadTable, 'all');
    end
    overloadOfTruck = sum((totalTruckLoadTable - maxCapacityTable) .* (totalTruckLoadTable > maxCapacityTable), 'all');
end

% 无人机容量约束
function [overloadOfDrone] = getOverloadOfDrone(droneLoadTableCell, model)
    maxCapacityTable = zeros(model.numOfDrone, model.numOfDroneMaxDepart) + model.maxCapacityOfDrone;

    overloadArray = zeros(size(droneLoadTableCell));
    for i = 1: numel(droneLoadTableCell)
        droneLoadTable = droneLoadTableCell{i};
        overloadArray(i) = sum((droneLoadTable - maxCapacityTable) .* (droneLoadTable > maxCapacityTable), 'all');
    end
    overloadOfDrone = sum(overloadArray, 'all');
end

% 卡车最大行驶距离约束
function [overDistanceOfTruck] = getOverDistanceOfTruck(truckPathDistanceTable, model)
    maxDistanceTable = zeros(size(truckPathDistanceTable)) + model.maxDistanceOfTruck;
    overDistanceOfTruck = sum((truckPathDistanceTable - maxDistanceTable) .* (truckPathDistanceTable > maxDistanceTable), 'all');
end

% 无人机最大行驶距离约束
function [overDistanceOfDrone] = getOverDistanceOfDrone(dronePathDistanceTableCell, model)
    maxDistanceTable = zeros(model.numOfDrone, model.numOfDroneMaxDepart) + model.maxDistanceOfDrone;

    overDistanceArray = zeros(size(dronePathDistanceTableCell));
    for i = 1: numel(dronePathDistanceTableCell)
        dronePathDistanceTable = dronePathDistanceTableCell{i};
        overDistanceArray(i) = sum((dronePathDistanceTable - maxDistanceTable) .* (dronePathDistanceTable > maxDistanceTable), 'all');
    end
    overDistanceOfDrone = sum(overDistanceArray, 'all');
end

% 无人机起点终点不能为同一个点
function [numOfSamePoint] = getNumOfSamePoint(realDronePathTableCell, model)
    numOfSamePoint = 0;
    for truckId = 1: model.numOfTruck
        realDronePathTable = realDronePathTableCell{truckId};
        for droneId = 1: model.numOfDrone
            realDronePathOfOne = realDronePathTable(droneId, :);
            for departId = 1: model.numOfDroneMaxDepart
                dronePath = realDronePathOfOne{departId};
                if ~isempty(dronePath)
                    if dronePath(1) == dronePath(end)
                        numOfSamePoint = numOfSamePoint + 1;
                    end
                end
            end
        end
    end
end

% 卡车总路程成本Cost1
function [Cost1] = getCost1(truckPathDistanceTable, model)
    Cost1 = sum(truckPathDistanceTable, 'all');
end

% 无人机总路程成本Cost2
function [Cost2] = getCost2(dronePathDistanceTableCell, model)
    Cost2 = 0;
    for i = 1: numel(dronePathDistanceTableCell)
        dronePathDistanceTable = dronePathDistanceTableCell{i};
        Cost2 = Cost2 + sum(dronePathDistanceTable, 'all');
    end
end

% 完成全部任务时刻
function [Cost3, completionTimeArray] = getCost3(truckPathTable, realDronePathTableCell, model)
    numOfTruck = model.numOfTruck;
    completionTimeArray = zeros(numOfTruck, 1);
    for truckId = 1: numOfTruck
        truckPath = truckPathTable{truckId};
        realDronePathTable = realDronePathTableCell{truckId};
        [completionTime] = getMaxFinishTime({truckPath}, {realDronePathTable}, model);
        completionTimeArray(truckId) = completionTime;
    end
    Cost3 = max(completionTimeArray);
end

function [Cost1, Cost2, Cost3, overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint] = getAllCost(individual, model)
    [pathTable] = getPathTable(individual, model);
    [truckPathTable, planDronePathTableCell] = getTruckAndDronePathTable(pathTable, model);
    [realDronePathTableCell] = getRealDronePathTableCell(truckPathTable, planDronePathTableCell, model);
    
    [truckPathDistanceTable, truckStateTable, truckLoadTable] = getTruckPathDistanceTable(truckPathTable, model);
    [dronePathDistanceTableCell, droneStateTableCell, droneLoadTableCell] = getDronePathDistanceTableCell(realDronePathTableCell, model);

    [overloadOfTruck] = getOverloadOfTruck(truckLoadTable, droneLoadTableCell, model);      % 卡车容量约束
    [overloadOfDrone] = getOverloadOfDrone(droneLoadTableCell, model);                      % 无人机容量约束
    [overDistanceOfTruck] = getOverDistanceOfTruck(truckPathDistanceTable, model);          % 卡车最大行驶距离约束
    [overDistanceOfDrone] = getOverDistanceOfDrone(dronePathDistanceTableCell, model);      % 无人机最大行驶距离约束
    [numOfSamePoint] = getNumOfSamePoint(realDronePathTableCell, model);                    % 无人机起点终点不能为同一个点

    [Cost1] = getCost1(truckPathDistanceTable, model);                                              % 卡车总路程Cost1
    [Cost2] = getCost2(dronePathDistanceTableCell, model);                                          % 无人机总路程Cost2
    [Cost3] = getCost3(truckPathTable, realDronePathTableCell, model);                              % 完成时间Cost3
end

function [Cost1, Cost2, Cost3, overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint] = getAllCost2(truckPathTable, realDronePathTableCell, model)
    [truckPathDistanceTable, truckStateTable, truckLoadTable] = getTruckPathDistanceTable(truckPathTable, model);
    [dronePathDistanceTableCell, droneStateTableCell, droneLoadTableCell] = getDronePathDistanceTableCell(realDronePathTableCell, model);

    [overloadOfTruck] = getOverloadOfTruck(truckLoadTable, droneLoadTableCell, model);      % 卡车容量约束
    [overloadOfDrone] = getOverloadOfDrone(droneLoadTableCell, model);                      % 无人机容量约束
    [overDistanceOfTruck] = getOverDistanceOfTruck(truckPathDistanceTable, model);          % 卡车最大行驶距离约束
    [overDistanceOfDrone] = getOverDistanceOfDrone(dronePathDistanceTableCell, model);      % 无人机最大行驶距离约束
    [numOfSamePoint] = getNumOfSamePoint(realDronePathTableCell, model);                    % 无人机起点终点不能为同一个点

    [Cost1] = getCost1(truckPathDistanceTable, model);                                              % 卡车总路程Cost1
    [Cost2] = getCost2(dronePathDistanceTableCell, model);                                          % 无人机总路程Cost2
    [Cost3] = getCost3(truckPathTable, realDronePathTableCell, model);                              % 完成时间Cost3
end

% 适应度函数
function [individualFitness] = getIndividualFitness(individual, model)
    [Cost1, Cost2, Cost3, overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint] = getAllCost(individual, model);
    penalty = (overloadOfTruck + overloadOfDrone + overDistanceOfTruck + overDistanceOfDrone + numOfSamePoint) * model.penaltyFactor;
    individualFitness = - sum(model.weightOfObjs .* [Cost1 Cost2 Cost3]) - penalty * 1;
end

function printIndividual(individual, model)
    [Cost1, Cost2, Cost3, overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint] = getAllCost(individual, model);
    [individualFitness] = getIndividualFitness(individual, model);
    fprintf('卡车容量约束:%.2f 无人机容量约束:%.2f 卡车最大行驶距离约束:%.2f 无人机最大行驶距离约束:%.2f 无人机同一起终点约束:%.2f\n', overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint);
    fprintf('卡车总路程C1:%.4f 无人机总路程C2:%.4f 完成时间C3:%.4f 目标函数:%.2f\n', Cost1, Cost2, Cost3, -individualFitness);
end

function printIndividual2(truckPathTable, realDronePathTableCell, model)
    [Cost1, Cost2, Cost3, overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint] = getAllCost2(truckPathTable, realDronePathTableCell, model);
   
    % fprintf('卡车容量约束:%.2f 无人机容量约束:%.2f 卡车最大行驶距离约束:%.2f 无人机最大行驶距离约束:%.2f 无人机同一起终点约束:%.2f\n', overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint);
    % fprintf('卡车总路程C1:%.4f 无人机总路程C2:%.4f 完成时间C3:%.4f\n', Cost1, Cost2, Cost3);
    n1 = 0;
    n2 = 0;
    n3 = 0;
    for i = 1: numel(truckPathTable)
        truckPath = truckPathTable{i};
        n1 = n1 + numel(truckPath) - 2;

        realDronePathTable = realDronePathTableCell{i};
        for j = 1: numel(realDronePathTable)
            dronePath = realDronePathTable{j};
            if numel(dronePath) > 2
                n2 = n2 + numel(dronePath) - 2;
                n3 = n3 + 1;
            end
        end

    end
    fprintf('卡车访问客户数\t%d\t无人机访问客户数\t%d\t无人机起飞次数\t%d\n', n1, n2, n3);
end



%% 绘图
function showIndividual(individual, type, model)
    hold on;
    drawPoints(model);
    drawPath(individual, type, model);
end

function showIndividual2(truckPathTable, realDronePathTableCell, type, model)
    hold on;
    drawPoints(model);
    drawPath2(truckPathTable, realDronePathTableCell, type, model);
end

function drawPoints(model)
    coordOfPoint = [model.coordOfCustomer; model.coordOfCentre];
    idsOfCustomer = 1: model.numOfCustomer;
    idsOfCentre = (1: model.numOfCentre) + model.numOfCustomer;
    xCoord = coordOfPoint(:, 1);
    yCoord = coordOfPoint(:, 2);
    plot(xCoord(idsOfCustomer), yCoord(idsOfCustomer), 'ob', 'MarkerSize', 5,'LineWidth', 1, 'DisplayName', '顾客');
    plot(xCoord(idsOfCentre), yCoord(idsOfCentre), '*r', 'MarkerSize', 20,'LineWidth', 2, 'DisplayName', '仓库');
    legend('Location', 'northeastoutside');
    for i = 1 : size(coordOfPoint, 1)
        text(xCoord(i), yCoord(i),['   ' num2str(i)], 'FontSize', 8);
    end
end

function drawPath(individual, type, model)
    coordOfPoint = [model.coordOfCustomer; repmat(model.coordOfCentre, [model.numOfTruck, 1])];
    xCoord = coordOfPoint(:, 1);
    yCoord = coordOfPoint(:, 2);
    colors = {'r', 'g', 'b', 'y', 'm', 'k'};

    [pathTable] = getPathTable(individual, model);
    [truckPathTable, planDronePathTableCell] = getTruckAndDronePathTable(pathTable, model);
    [realDronePathTableCell] = getRealDronePathTableCell(truckPathTable, planDronePathTableCell, model);
    
    [truckPathDistanceTable, truckStateTable, truckLoadTable] = getTruckPathDistanceTable(truckPathTable, model);
    [dronePathDistanceTableCell, droneStateTableCell, droneLoadTableCell] = getDronePathDistanceTableCell(realDronePathTableCell, model);
    [truckDroneTimeResultCell] = getTruckDroneTimeResultCell(truckPathTable, realDronePathTableCell, model);
    
    % 卡车路线
    k = 0;
    for truckId = 1: length(truckPathTable)
        truckPath = truckPathTable{truckId};
        truckDroneTimeResult = truckDroneTimeResultCell{truckId};
        truckArrivalTimeArray = truckDroneTimeResult.truckArrivalTimeArray;
        truckLeaveTimeArray = truckDroneTimeResult.truckLeaveTimeArray;
        if length(truckPath) > 2
            k = k + 1;
            if k == 1
                plot(xCoord(truckPath), yCoord(truckPath), '-', 'LineWidth', 1, 'DisplayName', '卡车路线');
            else
                plot(xCoord(truckPath), yCoord(truckPath), '-', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            tDistance = truckPathDistanceTable(truckId);
            tLoad = truckLoadTable(truckId);
            tWorkTime = truckLeaveTimeArray(end) - truckArrivalTimeArray(truckId);
            fprintf('卡车%d 距离%.2f 负载%.2f 工作时长%.2f 路线:%s\n', truckId, tDistance, tLoad, tWorkTime, num2str(truckPath));
            fprintf('卡车每点到达时刻:%s\n', num2str(truckArrivalTimeArray, '%.2f\t'));
            fprintf('卡车每点离开时刻:%s\n', num2str(truckLeaveTimeArray, '%.2f\t'));
            fprintf('卡车每点等待时长:%s\n', num2str(truckLeaveTimeArray - truckArrivalTimeArray, '%.2f\t'));
        end
    end

    % 无人机路线
    k = 0;
    for truckId = 1: numel(realDronePathTableCell)
        dronePathTable = realDronePathTableCell{truckId};
        dronePathDistanceTable = dronePathDistanceTableCell{truckId};
        droneLoadTable = droneLoadTableCell{truckId};

        truckDroneTimeResult = truckDroneTimeResultCell{truckId};
        realDroneArrivalTimeTable = truckDroneTimeResult.realDroneArrivalTimeTable;
        realDroneLeaveTimeTable = truckDroneTimeResult.realDroneLeaveTimeTable;
        realDroneFinishTimeTable = truckDroneTimeResult.realDroneFinishTimeTable;
        realDroneWaitingTimeTable = truckDroneTimeResult.realDroneWaitingTimeTable;

        for i = 1: size(dronePathTable, 1)
            for j = 1: size(dronePathTable, 2)
                dronePath = dronePathTable{i, j};
                if length(dronePath) > 2
                    k = k + 1;
                    if k == 1
                        plot(xCoord(dronePath), yCoord(dronePath), '--', 'color', colors{i}, 'LineWidth', 1, 'DisplayName', '无人机路线');
                    else
                        plot(xCoord(dronePath), yCoord(dronePath), '--', 'color', colors{i}, 'LineWidth', 1, 'HandleVisibility', 'off');
                    end
                    dDistance = dronePathDistanceTable(i, j);
                    dLoad = droneLoadTable(i, j);
                    t1 = realDroneArrivalTimeTable(i, j);
                    t2 = realDroneLeaveTimeTable(i, j);
                    t3 = realDroneFinishTimeTable(i, j);
                    t4 = realDroneWaitingTimeTable(i, j);
                    fprintf('卡车%d\t无人机%d\t第%02d次\t距离%.2f\t负载%.2f\t到达时刻%-6.2f\t离开时刻%-6.2f\t完成时刻%-6.2f\t等待时长%-6.2f\t路线:%s\n', truckId, i, j, dDistance, dLoad, t1, t2, t3, t4, num2str(dronePath));
                end
            end
        end
    end

    if type ~= 1
        set(gcf, 'Position', get(0, 'ScreenSize'));
        dateString = datestr(now, 'yyyymmdd-HHMM');
        savePath = sprintf('./result/png/problem%02d-%s.png', model.problemId, date);
        ensureDirectory(savePath);
        saveas(gcf, savePath, 'png');
    end
end

function saveResult(individual, model)
    model.printSubmitResult(individual, 0, model);

    dateString = datestr(now, 'yyyymmdd-HHMM');
    savePath = sprintf('./result/txt/problem%02d-%s.txt', model.problemId, dateString);
    ensureDirectory(savePath);
    fid = fopen(savePath, 'w');

    fprintf(fid, '个体:\n');
    fprintf(fid, '%s\n', num2str(individual));

    [pathTable] = getPathTable(individual, model);
    [truckPathTable, planDronePathTableCell] = getTruckAndDronePathTable(pathTable, model);
    [realDronePathTableCell] = getRealDronePathTableCell(truckPathTable, planDronePathTableCell, model);
    [truckPathDistanceTable, truckStateTable, truckLoadTable] = getTruckPathDistanceTable(truckPathTable, model);
    [dronePathDistanceTableCell, droneStateTableCell, droneLoadTableCell] = getDronePathDistanceTableCell(realDronePathTableCell, model);
    
    [truckDroneTimeResultCell] = getTruckDroneTimeResultCell(truckPathTable, realDronePathTableCell, model);    % 完成时间Cost3
    
    % 卡车路线
    for truckId = 1: length(truckPathTable)
        truckPath = truckPathTable{truckId};
        truckDroneTimeResult = truckDroneTimeResultCell{truckId};
        truckArrivalTimeArray = truckDroneTimeResult.truckArrivalTimeArray;
        truckLeaveTimeArray = truckDroneTimeResult.truckLeaveTimeArray;
        if length(truckPath) > 2
            tDistance = truckPathDistanceTable(truckId);
            tLoad = truckLoadTable(truckId);
            tWorkTime = truckLeaveTimeArray(end) - truckArrivalTimeArray(truckId);
            fprintf(fid, '卡车%d 距离%.2f 负载%.2f 工作时长%.2f 路线:%s\n', truckId, tDistance, tLoad, tWorkTime, num2str(truckPath));
            fprintf(fid, '卡车每点到达时刻:%s\n', num2str(truckArrivalTimeArray, '%.2f\t'));
            fprintf(fid, '卡车每点离开时刻:%s\n', num2str(truckLeaveTimeArray, '%.2f\t'));
            fprintf(fid, '卡车每点等待时长:%s\n', num2str(truckLeaveTimeArray - truckArrivalTimeArray, '%.2f\t'));
        end
    end

    % 无人机路线
    k = 0;
    for truckId = 1: numel(realDronePathTableCell)
        dronePathTable = realDronePathTableCell{truckId};
        dronePathDistanceTable = dronePathDistanceTableCell{truckId};
        droneLoadTable = droneLoadTableCell{truckId};

        truckDroneTimeResult = truckDroneTimeResultCell{truckId};
        realDroneArrivalTimeTable = truckDroneTimeResult.realDroneArrivalTimeTable;
        realDroneLeaveTimeTable = truckDroneTimeResult.realDroneLeaveTimeTable;
        realDroneFinishTimeTable = truckDroneTimeResult.realDroneFinishTimeTable;
        realDroneWaitingTimeTable = truckDroneTimeResult.realDroneWaitingTimeTable;

        for i = 1: size(dronePathTable, 1)
            for j = 1: size(dronePathTable, 2)
                dronePath = dronePathTable{i, j};
                if length(dronePath) > 2
                    dDistance = dronePathDistanceTable(i, j);
                    dLoad = droneLoadTable(i, j);
                    t1 = realDroneArrivalTimeTable(i, j);
                    t2 = realDroneLeaveTimeTable(i, j);
                    t3 = realDroneFinishTimeTable(i, j);
                    t4 = realDroneWaitingTimeTable(i, j);
                    fprintf(fid, '卡车%d\t无人机%d\t第%02d次\t距离%.2f\t负载%.2f\t到达时刻%-6.2f\t离开时刻%-6.2f\t完成时刻%-6.2f\t等待时长%-6.2f\t路线:%s\n', truckId, i, j, dDistance, dLoad, t1, t2, t3, t4, num2str(dronePath));
                end
            end
        end
    end

    [Cost1, Cost2, Cost3, overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint] = getAllCost(individual, model);
    [individualFitness] = getIndividualFitness(individual, model);
    fprintf(fid, '卡车容量约束:%.2f 无人机容量约束:%.2f 卡车最大行驶距离约束:%.2f 无人机最大行驶距离约束:%.2f 无人机同一起终点约束:%.2f\n', overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone, numOfSamePoint);
    fprintf(fid, '卡车总路程C1:%.4f 无人机总路程C2:%.4f 完成时间C3:%.4f 目标函数:%.2f\n', Cost1, Cost2, Cost3, -individualFitness);
    fclose(fid);
end

%% 比赛提交输出结果
function [submitTruckPathTable, submitRealDronePathTableCell] = getSubmitTruckPathTable(individual, model)
    [pathTable] = getPathTable(individual, model);
    [truckPathTable, planDronePathTableCell] = getTruckAndDronePathTable(pathTable, model);
    [realDronePathTableCell] = getRealDronePathTableCell(truckPathTable, planDronePathTableCell, model);

    [newIndividual] = truckDronePathTableToIndividual(truckPathTable, realDronePathTableCell, model);
    % 校验
    if sum(sort(newIndividual) - model.sequence ~= 0) == 0
        fprintf('合法转换\n');
    end


    submitTruckPathTable = truckPathTable;
    for truckId = 1: model.numOfTruck
        truckPath = submitTruckPathTable{truckId};
        truckPath(1) = 0;
        truckPath(end) = 0;
        submitTruckPathTable{truckId} = truckPath;
    end

    submitRealDronePathTableCell = realDronePathTableCell;
    for truckId = 1: model.numOfTruck
        truckPath = truckPathTable{truckId};
        realDronePathTable = submitRealDronePathTableCell{truckId};
        for droneId = 1: model.numOfDrone
            realDronePathArray = realDronePathTable(droneId, :);
            % 无人机每次飞行的起飞点、降落点，对应在车辆路线上的索引
            [droneDepartPointIndexs, droneFallPointIndexs] = getDroneDepartFallPointIndexs(realDronePathArray, truckPath);
            for departId = 1: length(droneDepartPointIndexs)
                dronePath = realDronePathArray{departId};
                dronePath(1) = droneDepartPointIndexs(departId) - 1;
                dronePath(end) = droneFallPointIndexs(departId) - 1;
                realDronePathArray{departId} = dronePath;
            end
            realDronePathTable(droneId, :) = realDronePathArray;
        end
        submitRealDronePathTableCell{truckId} = realDronePathTable;
    end
end

function savePathCellArray(fid, pathCellArray)
    k = 0;
    fprintf(fid, '[');
    for i = 1: numel(pathCellArray)
        path = pathCellArray{i};
        str = mat2str(path);
        str = strrep(str, ' ', ', ');
        if length(path) >= 2
            k = k + 1;
            if i < numel(pathCellArray)
                if length(pathCellArray{i + 1}) >= 2
                    fprintf(fid, '%s, ', str);
                else
                    fprintf(fid, '%s', str);
                end
            else
                fprintf(fid, '%s', str);
            end
        end
    end
    fprintf(fid, ']\n');
end

function printSubmitResult(individual, type, model)
    [submitTruckPathTable, submitRealDronePathTableCell] = getSubmitTruckPathTable(individual, model);
    if type ~= 1
        dateString1 = datestr(now, 'yyyymmdd');
        dateString2 = datestr(now, 'yyyymmdd-HHMM');
        savePath = sprintf('./result/submit/%s/Instance%d-%s.txt', dateString1, model.problemId, dateString2);
        ensureDirectory(savePath);
        fid = fopen(savePath, 'w');
    else
        fid = 1;
    end
    fprintf(fid, 'truck_route = ');
    savePathCellArray(fid, submitTruckPathTable);
    k = 0;
    for truckId = 1: model.numOfTruck
        realDronePathTable = submitRealDronePathTableCell{truckId};
        for droneId = 1: model.numOfDrone
            realDronePathArray = realDronePathTable(droneId, :);
            k = k + 1;
            fprintf(fid, 'drone_routes%d = ', k);
            savePathCellArray(fid, realDronePathArray);
        end
    end

    if type ~= 1
        fclose(fid);
    end
end

%%
function drawPartPath(path, model)
    figure;
    hold on;
    drawPoints(model);
    coordOfPoint = [model.coordOfCustomer; repmat(model.coordOfCentre, [model.numOfTruck, 1])];
    xCoord = coordOfPoint(:, 1);
    yCoord = coordOfPoint(:, 2);
    plot(xCoord(path), yCoord(path), '-', 'LineWidth', 1);
end


function drawPath2(truckPathTable, realDronePathTableCell, type, model)
    coordOfPoint = [model.coordOfCustomer; repmat(model.coordOfCentre, [model.numOfTruck, 1])];
    xCoord = coordOfPoint(:, 1);
    yCoord = coordOfPoint(:, 2);
    colors = {'r', 'g', 'b', 'y', 'm', 'k'};

    

    [truckPathDistanceTable, truckStateTable, truckLoadTable] = getTruckPathDistanceTable(truckPathTable, model);
    [dronePathDistanceTableCell, droneStateTableCell, droneLoadTableCell] = getDronePathDistanceTableCell(realDronePathTableCell, model);
    [truckDroneTimeResultCell] = getTruckDroneTimeResultCell(truckPathTable, realDronePathTableCell, model);
    
    % 卡车路线
    k = 0;
    for truckId = 1: length(truckPathTable)
        truckPath = truckPathTable{truckId};
        truckDroneTimeResult = truckDroneTimeResultCell{truckId};
        truckArrivalTimeArray = truckDroneTimeResult.truckArrivalTimeArray;
        truckLeaveTimeArray = truckDroneTimeResult.truckLeaveTimeArray;
        if length(truckPath) > 2
            k = k + 1;
            if k == 1
                plot(xCoord(truckPath), yCoord(truckPath), '-', 'LineWidth', 1, 'DisplayName', '卡车路线');
            else
                plot(xCoord(truckPath), yCoord(truckPath), '-', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            tDistance = truckPathDistanceTable(truckId);
            tLoad = truckLoadTable(truckId);
            tWorkTime = truckLeaveTimeArray(end) - truckArrivalTimeArray(truckId);
            fprintf('卡车%d 距离%.2f 负载%.2f 工作时长%.2f 路线:%s\n', truckId, tDistance, tLoad, tWorkTime, num2str(truckPath));
            fprintf('卡车每点到达时刻:%s\n', num2str(truckArrivalTimeArray, '%.2f\t'));
            fprintf('卡车每点离开时刻:%s\n', num2str(truckLeaveTimeArray, '%.2f\t'));
            fprintf('卡车每点等待时长:%s\n', num2str(truckLeaveTimeArray - truckArrivalTimeArray, '%.2f\t'));
        end
    end

    % 无人机路线
    k = 0;
    for truckId = 1: numel(realDronePathTableCell)
        dronePathTable = realDronePathTableCell{truckId};
        dronePathDistanceTable = dronePathDistanceTableCell{truckId};
        droneLoadTable = droneLoadTableCell{truckId};

        truckDroneTimeResult = truckDroneTimeResultCell{truckId};
        realDroneArrivalTimeTable = truckDroneTimeResult.realDroneArrivalTimeTable;
        realDroneLeaveTimeTable = truckDroneTimeResult.realDroneLeaveTimeTable;
        realDroneFinishTimeTable = truckDroneTimeResult.realDroneFinishTimeTable;
        realDroneWaitingTimeTable = truckDroneTimeResult.realDroneWaitingTimeTable;

        for i = 1: size(dronePathTable, 1)
            for j = 1: size(dronePathTable, 2)
                dronePath = dronePathTable{i, j};
                if length(dronePath) > 2
                    k = k + 1;
                    if k == 1
                        plot(xCoord(dronePath), yCoord(dronePath), '--', 'color', colors{i}, 'LineWidth', 1, 'DisplayName', '无人机路线');
                    else
                        plot(xCoord(dronePath), yCoord(dronePath), '--', 'color', colors{i}, 'LineWidth', 1, 'HandleVisibility', 'off');
                    end
                    dDistance = dronePathDistanceTable(i, j);
                    dLoad = droneLoadTable(i, j);
                    t1 = realDroneArrivalTimeTable(i, j);
                    t2 = realDroneLeaveTimeTable(i, j);
                    t3 = realDroneFinishTimeTable(i, j);
                    t4 = realDroneWaitingTimeTable(i, j);
                    fprintf('卡车%d\t无人机%d\t第%02d次\t距离%.2f\t负载%.2f\t到达时刻%-6.2f\t离开时刻%-6.2f\t完成时刻%-6.2f\t等待时长%-6.2f\t路线:%s\n', truckId, i, j, dDistance, dLoad, t1, t2, t3, t4, num2str(dronePath));
                end
            end
        end
    end

    if type ~= 1
        set(gcf, 'Position', get(0, 'ScreenSize'));
        dateString = datestr(now, 'yyyymmdd-HHMM');
        savePath = sprintf('./result/png/problem%02d-%s.png', model.problemId, date);
        ensureDirectory(savePath);
        saveas(gcf, savePath, 'png');
    end
end


%% 比赛提交输出结果
function [submitTruckPathTable, submitRealDronePathTableCell] = getSubmitTruckPathTable2(truckPathTable, realDronePathTableCell, model)
    [newIndividual] = truckDronePathTableToIndividual(truckPathTable, realDronePathTableCell, model);
    % 校验
    if sum(sort(newIndividual) - model.sequence ~= 0) == 0
        fprintf('合法转换\n');
    end


    submitTruckPathTable = truckPathTable;
    for truckId = 1: model.numOfTruck
        truckPath = submitTruckPathTable{truckId};
        truckPath(1) = 0;
        truckPath(end) = 0;
        submitTruckPathTable{truckId} = truckPath;
    end

    submitRealDronePathTableCell = realDronePathTableCell;
    for truckId = 1: model.numOfTruck
        truckPath = truckPathTable{truckId};
        realDronePathTable = submitRealDronePathTableCell{truckId};
        for droneId = 1: model.numOfDrone
            realDronePathArray = realDronePathTable(droneId, :);
            % 无人机每次飞行的起飞点、降落点，对应在车辆路线上的索引
            [droneDepartPointIndexs, droneFallPointIndexs] = getDroneDepartFallPointIndexs(realDronePathArray, truckPath);
            for departId = 1: length(droneDepartPointIndexs)
                dronePath = realDronePathArray{departId};
                dronePath(1) = droneDepartPointIndexs(departId) - 1;
                dronePath(end) = droneFallPointIndexs(departId) - 1;
                realDronePathArray{departId} = dronePath;
            end
            realDronePathTable(droneId, :) = realDronePathArray;
        end
        submitRealDronePathTableCell{truckId} = realDronePathTable;
    end
end


function printSubmitResult2(truckPathTable, realDronePathTableCell, type, model)
    [submitTruckPathTable, submitRealDronePathTableCell] = getSubmitTruckPathTable2(truckPathTable, realDronePathTableCell, model);
    if type ~= 1
        dateString1 = datestr(now, 'yyyymmdd');
        dateString2 = datestr(now, 'yyyymmdd-HHMM');
        savePath = sprintf('./result/submit/%s/Instance%d-%s.txt', dateString1, model.problemId, dateString2);
        ensureDirectory(savePath);
        fid = fopen(savePath, 'w');
    else
        fid = 1;
    end
    fprintf(fid, 'truck_route = ');
    savePathCellArray(fid, submitTruckPathTable);
    k = 0;
    for truckId = 1: model.numOfTruck
        realDronePathTable = submitRealDronePathTableCell{truckId};
        for droneId = 1: model.numOfDrone
            realDronePathArray = realDronePathTable(droneId, :);
            k = k + 1;
            fprintf(fid, 'drone_routes%d = ', k);
            savePathCellArray(fid, realDronePathArray);
        end
    end

    if type ~= 1
        fclose(fid);
    end
end

function drawRealDronePathCell(truckPath, realDronePathCell, model)
    figure;
    hold on;
    coordOfPoint = [model.coordOfCustomer; repmat(model.coordOfCentre, [model.numOfTruck, 1])];
    xCoord = coordOfPoint(:, 1);
    yCoord = coordOfPoint(:, 2);
    idsOfCustomer = truckPath(2: end - 1);
    plot(xCoord(truckPath), yCoord(truckPath), '-', 'LineWidth', 1);
    for i = 1: numel(realDronePathCell)
        dPath = realDronePathCell{i};
        plot(xCoord(dPath), yCoord(dPath), '--', 'LineWidth', 1);
        idsOfCustomer = [idsOfCustomer dPath(2: end - 1)];
    end


    
    idsOfCentre = (1: model.numOfCentre) + model.numOfCustomer;
    xCoord = coordOfPoint(:, 1);
    yCoord = coordOfPoint(:, 2);
    plot(xCoord(idsOfCustomer), yCoord(idsOfCustomer), 'ob', 'MarkerSize', 5,'LineWidth', 1);
    plot(xCoord(idsOfCentre), yCoord(idsOfCentre), '*r', 'MarkerSize', 20,'LineWidth', 2);
    for i = 1 : length(idsOfCustomer)
        text(xCoord(idsOfCustomer(i)), yCoord(idsOfCustomer(i)),['   ' num2str(idsOfCustomer(i))], 'FontSize', 8);
    end

end


