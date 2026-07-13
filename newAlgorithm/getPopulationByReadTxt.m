function [population] = getPopulationByReadTxt(folderPath, model)
    fileExtension = '*.txt';
    files = dir(fullfile(folderPath, fileExtension));
    
    population = zeros(numel(files), model.numOfDecVariables);
    for i = 1: length(files)
        resultFile = [folderPath files(i).name];
        fprintf('%s\n', resultFile);
        [truckRoutes, droneRoutes] = readTruckDroneRoutes(resultFile);
        [truckPathTable, realDronePathTableCell] = truckRoutesToTruckPathTable(truckRoutes, droneRoutes, model);
        [individual] = truckDronePathTableToIndividual(truckPathTable, realDronePathTableCell, model);
        population(i, :) = individual;
    end
    [population] = repairOperation(population, model);
end
