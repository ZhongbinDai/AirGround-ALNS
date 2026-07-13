%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 从提交结果中找最优结果

problemId = 2;
numOfDroneMaxDepart = 10;
[model] = initModel3(problemId, numOfDroneMaxDepart);

folderPath = sprintf('H:/提交记录/20251018实验结果/%02d/', problemId);
% [population] = getPopulationByReadTxt(folderPath, model);

fileExtension = '*.txt';
files = dir(fullfile(folderPath, fileExtension));

population = zeros(numel(files), model.numOfDecVariables);

popFitness1 = zeros(numel(files), 1);
for i = 1: length(files)
    resultFile = [folderPath files(i).name];
    [truckRoutes, droneRoutes] = readTruckDroneRoutes(resultFile);
    [truckPathTable, realDronePathTableCell] = truckRoutesToTruckPathTable(truckRoutes, droneRoutes, model);
    [individual] = truckDronePathTableToIndividual(truckPathTable, realDronePathTableCell, model);
    population(i, :) = individual;

    [Cost3] = model.getCost3(truckPathTable, realDronePathTableCell, model);                              % 完成时间Cost3
    popFitness1(i) = Cost3;
end

[population] = repairOperation(population, model);
[popFitness2] = - getFitness(population, model);
diff = popFitness1 - popFitness2;


[minF, minIndex] = min(popFitness1);
fprintf('Cost%d min=%.6f\n', problemId, minF)
fprintf('文件: %s\n', files(minIndex).name);


[newPopulation] = perturbationOperation(population, model);



