function [newIndividual] = disturbOperationStrong(individual, j)
    numOfDecVariables = length(individual);
    newIndividual = individual;
    
    rJ = randperm(numOfDecVariables, 1);
    newIndividual([j, rJ]) = individual([rJ, j]);
end


