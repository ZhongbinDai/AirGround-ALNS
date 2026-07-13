function [newPopulation] = evolvePopulationGWO(a, alphaIndividual, betaIndividual, deltaIndividual, population)
    newPopulation = zeros(size(population));
    populationSize = size(population, 1);
    for i = 1 : populationSize
        individual = population(i, :);
        newIndividual = evolveIndividualGWO(a, alphaIndividual, betaIndividual, deltaIndividual, individual);
        newPopulation(i, :) = newIndividual;
    end
end

