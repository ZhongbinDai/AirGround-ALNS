%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng('default');
problemId = 1;
numOfDroneMaxDepart = 10;
[model] = initModel3(problemId, numOfDroneMaxDepart);
%%
folderPath = sprintf('./汇总分类/%02d/', problemId);
[population] = getPopulationByReadTxt(folderPath, model);

individual = population(end, :);
% [newIndividual] = repairOperation2(individual, model);
fileName = sprintf('./汇总分类/%02d/Instance1-20251019-0111.txt', problemId);
[truckRoutes, droneRoutes] = readTruckDroneRoutes(fileName);
[truckPathTable, realDronePathTableCell] = truckRoutesToTruckPathTable(truckRoutes, droneRoutes, model);
[Cost] = model.getCost3(truckPathTable, realDronePathTableCell, model)


figure;
model.showIndividual(individual, 0, model);
% figure;
% model.showIndividual(newIndividual, 0, model);
model.printIndividual(individual, model);
% model.printIndividual(newIndividual, model);
