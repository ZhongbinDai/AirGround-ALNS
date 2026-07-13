function [popPart, popDestroyPart] = destroyPopOperationOfAlns(population, numOfDestroy, popDestroyOperatorIds, model)
    [populationSize, numOfDecVariables] = size(population);
    popPart = zeros(populationSize, numOfDecVariables - numOfDestroy);
    popDestroyPart = zeros(populationSize, numOfDestroy);
    for i = 1: populationSize
        destroyOperatorId = popDestroyOperatorIds(i);
        individual = population(i, :);
        [individualPart, destroyPart] = destroyOperationOfAlns(individual, numOfDestroy, destroyOperatorId, model);
        popPart(i, :) = individualPart;
        popDestroyPart(i, :) = destroyPart;
    end
end

