%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng('default');


fileName = './data/5.txt';
[model] = initModel3(fileName);



[individual] = model.initIndividual(model);
[pathTable] = model.getPathTable(individual, model);
[truckPathTable, planDronePathTableCell] = model.getTruckAndDronePathTable(pathTable, model);
[realDronePathTableCell] = model.getRealDronePathTableCell(truckPathTable, planDronePathTableCell, model);

[pathLengthTable1] = model.getPathLengthTable(realDronePathTableCell{1});
[pathLengthTable2] = model.getPathLengthTable(realDronePathTableCell{2});

[truckPathDistanceTable, truckStateTable, truckLoadTable] = model.getTruckPathDistanceTable(truckPathTable, model);
[dronePathDistanceTableCell, droneStateTableCell, droneLoadTableCell] = model.getDronePathDistanceTableCell(realDronePathTableCell, model);



[Cost1, Cost2, Cost3, overloadOfTruck, overloadOfDrone, overDistanceOfTruck, overDistanceOfDrone] = model.getAllCost(individual, model)
figure;
model.showIndividual(individual, model);

model.printIndividualOfCompetition(individual, model);
model.printIndividual(individual, model);
model.printIndividual(model.repairIndividual2(individual, model), model);

% for i = 1: size(population, 1)
%     individual = population(i, :);
%     model.sequence - sort(individual)
% 
% 
% end
% 



