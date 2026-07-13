%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));
numOfDroneMaxDepart = 5;
problemId = 3;
populationSize = 100;
for problemId = 3: 3
    [model] = initModel3(problemId, numOfDroneMaxDepart);
    %% 改进点
    beta = 2;                                                                   % 贪婪因子重要程度
    greedyEpsilon = 1;                                                          % 贪婪度
    
    randTruck = 0.8;
    % [population1] = initBestPopulation(populationSize, model);
    [population1] = initPopulationByGreedy2(populationSize, beta, greedyEpsilon, randTruck, model);
    
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
    boxplot(X)
    % boxplot(X, {'改进的初始化种群', '随机初始化种群'})
    ylabel('完成时间')
end


% 
% [bestIndividual1, bestFitness, avgFitness] = getBestIndividualAndFitness(population1, popFitness1);
% figure;
% model.showIndividual(bestIndividual1, 1, model);
% 
% 
% [bestIndividual4, bestFitness, avgFitness] = getBestIndividualAndFitness(population4, popFitness4);
% figure;
% model.showIndividual(bestIndividual4, 1, model);
% 
% %%
% beta = 2;                                                                   % 贪婪因子重要程度
% greedyEpsilon = 1;                                                          % 贪婪度
% [newPopulation] = initPopulationByGreedy2(1, beta, greedyEpsilon, 0.3, model);
% [newPopulation] = repairOperation(newPopulation, model);                % 修复种群，防止越界
% newPopFitness = getFitness(newPopulation, model);                       % 子代种群适应度
% 
% - max(newPopFitness)




