function [newPopulation] = repairPopOperationOfAlns(popPart, popDestroyPart, popRepairOperatorIds, model)
    [populationSize] = size(popPart, 1);
    numOfDecVariables = model.numOfDecVariables;
    newPopulation = zeros(populationSize, numOfDecVariables);
    for i = 1: populationSize
        individualPart = popPart(i, :);
        destroyPart = popDestroyPart(i, :);
        repairOperatorId = popRepairOperatorIds(i);
        newPopulation(i, :) = repairOperationOfAlns(individualPart, destroyPart, repairOperatorId, model);
    end
end

