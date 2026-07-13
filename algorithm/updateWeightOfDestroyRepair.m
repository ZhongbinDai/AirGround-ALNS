function [weightsOfDestroy, weightsOfRepair, population, popFitness] = updateWeightOfDestroyRepair(weightsOfDestroy, weightsOfRepair, population, popFitness, newPopulation, newPopFitness, bestFitness, popDestroyOperatorIds, popRepairOperatorIds, scoreSet, cntOfDestroy, cntOfRepair, lambdaRate, T)
    for i = 1: length(popFitness)
        individual = population(i, :);
        individualFitness = popFitness(i);
        newIndividual = newPopulation(i, :);
        newIndividualFitness = newPopFitness(i);
        destroyOperatorId = popDestroyOperatorIds(i); 
        repairOperatorId = popRepairOperatorIds(i);
        [weightsOfDestroy, weightsOfRepair, individual, individualFitness] = update(weightsOfDestroy, weightsOfRepair, individual, individualFitness, newIndividual, newIndividualFitness, bestFitness, destroyOperatorId, repairOperatorId, scoreSet, cntOfDestroy, cntOfRepair, lambdaRate, T);
        population(i, :) = individual;
        popFitness(i) = individualFitness;
    end
end


function [weightsOfDestroy, weightsOfRepair, individual, individualFitness] = update(weightsOfDestroy, weightsOfRepair, individual, individualFitness, newIndividual, newIndividualFitness, bestFitness, destroyOperatorId, repairOperatorId, scoreSet, cntOfDestroy, cntOfRepair, lambdaRate, T)
    if individualFitness <= newIndividualFitness
        % 测试解更优，更新当前解
        individual = newIndividual;
        individualFitness = newIndividualFitness;
        if bestFitness <= newIndividualFitness
            % 测试解为历史最优解，更新历史最优解，并设置最高的算子得分
            scoreOfD = scoreSet(4);
            scoreOfR = scoreSet(4);
        else
            % 测试解不是历史最优解，但优于当前解，设置第二高的算子得分
            scoreOfD = scoreSet(3);
            scoreOfR = scoreSet(3);
        end
    else
        if rand() < exp(newIndividualFitness - individualFitness) / T
        % 当前解优于测试解，但满足模拟退火逻辑，依然更新当前解，设置第三高的算子得分
            individual = newIndividual;
            individualFitness = newIndividualFitness;
            scoreOfD = scoreSet(2);
            scoreOfR = scoreSet(2);
        else
        % 当前解优于测试解，也不满足模拟退火逻辑，不更新当前解，设置最低的算子得分
            scoreOfD = scoreSet(1);
            scoreOfR = scoreSet(1);
        end
    end
%     scoreOfDestroy(destroyOperatorId) = 0 * scoreOfDestroy(destroyOperatorId) + scoreOfD;
%     scoreOfRepair(repairOperatorId) = 0 * scoreOfRepair(repairOperatorId) + scoreOfR;


    % 更新destroy算子/repair算子的权重
    weightsOfDestroy(destroyOperatorId) = updateWeightOfAlns(weightsOfDestroy(destroyOperatorId), scoreOfD, cntOfDestroy(destroyOperatorId), lambdaRate);
    weightsOfRepair(repairOperatorId) = updateWeightOfAlns(weightsOfRepair(repairOperatorId), scoreOfR, cntOfRepair(repairOperatorId), lambdaRate);                        
end
