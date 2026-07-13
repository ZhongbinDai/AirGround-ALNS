%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));
problemId = 3;
for problemId = 1: 10
numOfDroneMaxDepart = 10;
[model] = initModel3(problemId, numOfDroneMaxDepart);



%% 改进点
beta = 2;                                                                   % 贪婪因子重要程度
greedyEpsilon = 1;                                                          % 贪婪度
populationSize = 100;
randTruck = 0.5;
% [population1] = initPopulationByGreedy2(populationSize, beta, greedyEpsilon, randTruck, model);
[population1] = initBestPopulation(populationSize, model);

distanceMat = model.distanceMatOfDrone;
numOfDecVariables = model.numOfDecVariables;                                % 决策变量维度
Eta = 1 ./ distanceMat;                                                     % Eta为启发因子,这里设为距离的倒数
pheromoneMat = ones(numOfDecVariables, numOfDecVariables);                 	% pheromone为信息素矩阵
[population2] = initPopulationByGreedy(populationSize, distanceMat, beta, greedyEpsilon, model);
population3 = getPopulationOfAco(populationSize, numOfDecVariables, pheromoneMat, Eta, 1, 5, model);
population4 = initialPopulation(populationSize, model);                      % 初始化种群

population1 = repairOperation(population1, model);   
population2 = repairOperation(population2, model);   
population3 = repairOperation(population3, model);
population4 = repairOperation(population4, model);  

popFitness1 = -getFitness(population1, model);
popFitness2 = -getFitness(population2, model);
popFitness3 = -getFitness(population3, model);
popFitness4 = -getFitness(population4, model);

X = [popFitness1 popFitness2 popFitness3 popFitness4];
figure;
boxplot(X);

% iId = randi([1 populationSize]);
% model.printIndividual(population1(iId, :), model);
% model.printIndividual(population2(iId, :), model);
% model.printIndividual(population3(iId, :), model);
% model.printIndividual(population4(iId, :), model);
% 
% 
% iId = randi([1 populationSize]);
% individual1 = population1(iId, :);
% [pathTable] = model.getPathTable(individual1, model);
% % model.drawPartPath(pathTable{2, 1}, model);
% 
% [pathLengthTable] = model.getPathLengthTable(pathTable);
% sum(pathLengthTable(:, 1) - 2)
end

% iId = randi([1 populationSize]);
% individual1 = population1(iId, :);
% [pathTable] = model.getPathTable(individual1, model);
% % model.drawPartPath(pathTable{2, 1}, model);
% 
% [pathLengthTable] = model.getPathLengthTable(pathTable);
% sum(pathLengthTable(:, 1) - 2)
% % 1 个体初始化改进，尽量均衡， 且路线尽量优

% 2 单个体,模块化,交换、插入、逆序

% 3 最优化模块路径


