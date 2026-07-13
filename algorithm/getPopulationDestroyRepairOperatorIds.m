function [popDestroyOperatorIds, popRepairOperatorIds] = getPopulationDestroyRepairOperatorIds(populationSize, weightsOfDestroy, weightsOfRepair)
    popDestroyOperatorIds = zeros(populationSize, 1);
    popRepairOperatorIds = zeros(populationSize, 1);
    for i = 1: populationSize
        destroyOperatorId = randsample(1: length(weightsOfDestroy), 1, true, weightsOfDestroy / sum(weightsOfDestroy));
        repairOperatorId = randsample(1: length(weightsOfRepair), 1, true, weightsOfRepair / sum(weightsOfRepair));
        popDestroyOperatorIds(i) = destroyOperatorId;
        popRepairOperatorIds(i) = repairOperatorId;
    end
end

