function [newWolf] = evolveIndividualGWO(a, alphaWolf, betaWolf, deltaWolf, wolf)
    [newIndividual1] = updateIndividualStrategyGWO(a, alphaWolf, wolf);
    [newIndividual2] = updateIndividualStrategyGWO(a, betaWolf, newIndividual1);
    [newIndividual3] = updateIndividualStrategyGWO(a, deltaWolf, newIndividual2);
    
    newWolf = newIndividual3;
end


