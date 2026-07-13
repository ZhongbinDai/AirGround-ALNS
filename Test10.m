%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng('default');
problemId = 3;
numOfDroneMaxDepart = 5;
[model] = initModel3(problemId, numOfDroneMaxDepart);

% populationSize = 100;
% population = initialPopulation(populationSize, model);                      % 初始化种群



fileName = sprintf('./saveResult-2025-10-20/Copy_of_Instance%d.txt', problemId);
[truckRoutes, droneRoutes] = readTruckDroneRoutes(fileName);
[truckPathTable, realDronePathTableCell] = truckRoutesToTruckPathTable(truckRoutes, droneRoutes, model);
[Cost3, completionTimeArray] = model.getCost3(truckPathTable, realDronePathTableCell, model);

[individual] = model.truckDronePathTableToIndividual(truckPathTable, realDronePathTableCell, model);
[individualFitness] = model.getIndividualFitness(individual, model)






