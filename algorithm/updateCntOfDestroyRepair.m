function [cntOfDestroy, cntOfRepair] = updateCntOfDestroyRepair(cntOfDestroy, cntOfRepair, popDestroyOperatorIds, popRepairOperatorIds)
    for i = 1: length(popDestroyOperatorIds)
        destroyOperatorId = popDestroyOperatorIds(i);
        repairOperatorId = popRepairOperatorIds(i);
        cntOfDestroy(destroyOperatorId) = cntOfDestroy(destroyOperatorId) + 1;
        cntOfRepair(repairOperatorId) = cntOfRepair(repairOperatorId) + 1;
    end
end

