function [newPopulation] = mutationOperationOfReal(population, mutationRate, model)
% 种群变异操作
    lower = model.lower;
    upper = model.upper;

    populationSize= size(population, 1);
    newPopulation = zeros(size(population));
    for i = 1 : populationSize
        individual = population(i, :);
        newPopulation(i, :) = mutateIndividual(individual, mutationRate, lower, upper);
    end
end

%% 个体变异
function [individual] = mutateIndividual(individual, mutationRate, lower, upper)
    D = size(individual, 2);
    
    for i = 1 : D
        if rand() < mutationRate
            individual(i) = round(rand() .* (upper(i) - lower(i)) + lower(i));
        end
    end
    
end
