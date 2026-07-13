function [newPopulation] = perturbationOperation(population, model)
    populationSize = size(population, 1);
    newPopulation = zeros(size(population));
    for i = 1 : populationSize
        individual = population(i, :);
        if rand() < 1 / 5
            newPopulation(i, :) = perturbationIndividual1(individual, model);
        elseif rand() < 2 / 5
            newPopulation(i, :) = perturbationIndividual2(individual, model);
        elseif rand() < 3 / 5
            newPopulation(i, :) = perturbationIndividual3(individual, model);
        elseif rand() < 3 / 5
            newPopulation(i, :) = perturbationIndividual4(individual, model);
        else
            individual = perturbationIndividual1(individual, model);
            newPopulation(i, :) = perturbationIndividual2(individual, model);
        end
    end
end

function [newIndividual] = perturbationIndividual4(individual, model)
    [pathTable] = model.getPathTable(individual, model);
    newPathTable = pathTable;
    
    n = size(pathTable, 1);
    rId = randi([1 n]);
    truckPath = pathTable{rId};
    newPathTable{rId} = truckPath(end: -1: 1);
    [newIndividual] = model.getPathTableToIndividual(newPathTable, model);
end


function [newIndividual] = perturbationIndividual3(individual, model)
    [pathTable] = model.getPathTable(individual, model);
    newPathTable = pathTable;

    [pathLengthTable] = model.getPathLengthTable(pathTable);
    minL = min(pathLengthTable(:));
    indexs = find(pathLengthTable(:) == minL);
    rTruckId = randi([1 model.numOfTruck]);
    rDroneId = indexs(randi([1 length(indexs)]));

    truckPath = pathTable{rTruckId};
    dronePath = pathTable{rDroneId};
    cIds = truckPath(2: end - 1);
    candidateIndexs = find(model.demandOfCustomer(cIds) <= model.maxCapacityOfDrone / 5) + 1;
    if length(candidateIndexs) >= 1
        rN = randi([1 3]);
        startI = randi([1 length(candidateIndexs)]);
        endI = startI + rN - 1;
        endI = min(endI, length(candidateIndexs));          % 防止越界
        goalIndexs = candidateIndexs(startI: endI)';
        goalCIds = truckPath(goalIndexs);                   % 选中的客户
        truckPath(goalIndexs) = [];                         % 剔除选中的客户
        dronePath = [dronePath(1: end - 1) goalCIds dronePath(end)];
        newPathTable{rTruckId} = truckPath;
        newPathTable{rDroneId} = dronePath;
    end
    [newIndividual] = model.getPathTableToIndividual(newPathTable, model);
end


function [newIndividual] = perturbationIndividual1(individual, model)
    rIndexs = sort(randperm(length(individual), 2));
    index1 = rIndexs(1);
    index2 = rIndexs(2);

    [newIndividual1] = swapOperation(individual, index1, index2);
    [newIndividual2] = reversedOperation(individual, index1, index2);
    [newIndividual3] = insertOperation(individual, index1, index2);
    newPopulation = [newIndividual1; newIndividual2; newIndividual3];       % 修复种群
    newPopulation = repairOperation(newPopulation, model);          	    % 修复种群
    newPopFitness = getFitness(newPopulation, model);                       % 计算种群适应度
    [~, Index] = max(newPopFitness);
    newIndividual = newPopulation(Index, :);
end

function [newIndividual] = perturbationIndividual2(individual, model)
    [pathTable] = model.getPathTable(individual, model);
    newPathTable = pathTable;
    pathArray = pathTable(:)';
    
    rIndexs = sort(randperm(length(pathArray), 2));
    index1 = rIndexs(1);
    index2 = rIndexs(2);


    newPathArray = swapOperation(pathArray, index1, index2);
    newPathTable(:) = newPathArray;
    [newIndividual1] = model.getPathTableToIndividual(newPathTable, model);
    newPathArray = reversedOperation(pathArray, index1, index2);
    newPathTable(:) = newPathArray;
    [newIndividual2] = model.getPathTableToIndividual(newPathTable, model);
    newPathArray = insertOperation2(pathArray, index1, index2);
    newPathTable(:) = newPathArray;
    [newIndividual3] = model.getPathTableToIndividual(newPathTable, model);

    newPopulation = [newIndividual1; newIndividual2; newIndividual3];       % 修复种群
    newPopulation = repairOperation(newPopulation, model);          	    % 修复种群
    newPopFitness = getFitness(newPopulation, model);                       % 计算种群适应度
    [~, Index] = max(newPopFitness);
    newIndividual = newPopulation(Index, :);
end

function [newIndividual] = swapOperation(individual, I, J)
    newIndividual = individual;
    newIndividual([I, J]) = individual([J, I]);
end

function [newIndividual] = reversedOperation(individual, I, J)
    newIndividual = individual;
    s = sort([I, J]);
    r0 = s(1);
    r1 = s(2); 
    newIndividual(r0: r1) = individual(r1: -1: r0);                         % r0-r1间元素倒序
end

function [newIndividual] = insertOperation(individual, I, J)
    newIndividual = individual;
    p1 = individual(I);
    p2 = individual(J);

    newIndividual(I) = -1;
    R = J;
    if rand() < 0.5
        R = J + 1;
    end
    newIndividual = [newIndividual(1: R - 1) p1 newIndividual(R: end)];
    index0 = newIndividual == -1;
    newIndividual(index0) = [];
end

function [newSeq] = insertOperation2(seq, I, J)
    newSeq = seq;
    p1 = seq(I);
    p2 = seq(J);

    newSeq{I} = [];
    R = J;
    if rand() < 0.5
        R = J + 1;
    end
    newSeq = [newSeq(1: R - 1) p1 newSeq(R: end)];

    for i = 1: numel(newSeq)
        if isempty(newSeq{i})
            newSeq(I) = [];
            break;
        end
    end
end


