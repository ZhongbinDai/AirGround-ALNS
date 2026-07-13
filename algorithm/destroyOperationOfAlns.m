function [individualPart, destroyPart] = destroyOperationOfAlns(individual, numOfDestroy, operatorId, model)
    if operatorId == 1
        [individualPart, destroyPart] = destroyByRandom(individual, numOfDestroy);
    elseif operatorId == 2
        [individualPart, destroyPart] = destroyByContinue(individual, numOfDestroy);
    else
        [individualPart, destroyPart] = destroyByGreed(individual, numOfDestroy, model);
    end
end

% 随机删除numOfDestroy个点
function [individualPart, destroyPart] = destroyByRandom(individual, numOfDestroy)
    N = length(individual);
    indexs = randperm(N, numOfDestroy);
    
    destroyPart = individual(indexs);
    individualPart = individual;
    individualPart(indexs) = [];
end

% 随机删除连续的numOfDestroy个点
function [individualPart, destroyPart] = destroyByContinue(individual, numOfDestroy)
    N = length(individual);
    index1 = randperm(N - numOfDestroy + 1, 1);
    index2 = index1 + numOfDestroy - 1;
    indexs = index1: index2;
    destroyPart = individual(indexs);
    individualPart = individual;
    individualPart(indexs) = [];
end

% 删除距离最大的numOfDestroy个点
function [individualPart, destroyPart] = destroyByGreed(individual, numOfDestroy, model)
    distanceMat = model.distanceMatOfDrone;
    distanceMatOfTruck = model.distanceMatOfTruck;

    distanceArray = zeros(size(individual));
    for i = 1: length(individual) - 1
        distanceArray(i) = distanceMat(individual(i), individual(i + 1));
    end
    
    % todo

    % 待优化
    numOfCustomer = model.numOfCustomer;                                    % 需求点数量
    centreIndex = find(individual > numOfCustomer);                         % 查询供应中心所在序列位置
    distanceArray(i) = distanceMat(individual(end), individual(centreIndex(end)));
    
    [~, indexs] = maxk(distanceArray, numOfDestroy);
    destroyPart = individual(indexs);
    individualPart = individual;
    individualPart(indexs) = [];
end


