function [newIndividual] = updateIndividualStrategyGWO(a, eliteIndividual, individual)
    numOfDecVariables = length(individual);
    J = randperm(numOfDecVariables, 1);
    if rand() < a
        [newIndividual] = learnOperation(individual, eliteIndividual, J);
    else
        [newIndividual] = disturbOperationWeak(eliteIndividual, J);
    end
end

