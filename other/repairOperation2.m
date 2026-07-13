function [newPopulation] = repairOperation2(population, model)
% 修复违反约束个体
    newPopulation = zeros(size(population));
    populationSize = size(population, 1);
    for i = 1 : populationSize
        % fprintf('%d\n', i);
        individual = population(i, :);
        newPopulation(i, :) = repairIndividual3(individual, model);
    end
end

function [newIndividual] = repairIndividual3(individual, model)
    [pathTable] = model.getPathTable(individual, model);
    newPathTable = pathTable;
    numOfTruck = size(pathTable, 1);
    
    for truckId = 1: numOfTruck
        if rand() < 0.5
            truckPath = pathTable{truckId};
            if length(truckPath) > 3
                sequence = truckPath(1: end - 1);
                coordOfPoints = model.coord(sequence, :);
                pathTemp = solveTSP(coordOfPoints);
                newTruckPath = sequence(pathTemp);
                if rand() < 0.5
                    newTruckPath = newTruckPath(end: -1: 1);
                end
                newPathTable{truckId} = newTruckPath;
            end
        end
    end
    [newIndividual] = model.getPathTableToIndividual(newPathTable, model);
end


