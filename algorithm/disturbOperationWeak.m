function [newIndividual] = disturbOperationWeak(individual, j)
    numOfDecVariables = length(individual);
    newIndividual = individual;
    
    rJ = randperm(numOfDecVariables, 1);
    s = sort([j rJ]);
    r0 = s(1);
    r1 = s(2); 
    newIndividual(r0: r1) = individual(r1:-1:r0);          % r0-r1¼äÔªËØµ¹Ðò
end

