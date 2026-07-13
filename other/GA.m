% -------------------------------------------------------------------------
% 遗传算法求解
% @作者：冰中呆
% @邮箱：1209805090@qq.com
% @时间：2025.10.20
% -------------------------------------------------------------------------

%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rand('state', 0);
populationSize = 100;                                                      	% 种群规模
maxGeneration = 1000;                                                      	% 最大进化代数
% crossoverRate = 0.6;                                                        % 交叉概率
% mutationRate = 0.1;                                                         % 变异概率
% stepSize = 0.1;                                                          	% 步长因子

proC = 0.6;                                                                 % 交叉概率
disC = 20;                                                                  % 模拟二元交叉的分布指数
proM = 1 / 50;                                                              % 变异概率
disM = 5;                                                                   % 多项式突变分布指数


%% 模型
problemId = 9;
numOfDroneMaxDepart = 10;
fileName = sprintf('./saveResult-2025-10-20 14/Instance%d.txt', problemId);
[model] = initModel4(problemId, numOfDroneMaxDepart, fileName);

mutationRate = 1 / model.numOfDecVariables;
%% 初始化
population = initialPopulation(populationSize, model);                      % 初始化种群
population(1, :) = model.individual0;
popFitness = getFitness(population, model);                                 % 计算种群适应度
numOfDecVariables = size(population, 2);                                    % 决策变量维度

bestIndividualSet = zeros(maxGeneration, numOfDecVariables);                % 每代最优个体集合
bestFitnessSet = zeros(maxGeneration, 1);                                   % 每代最高适应度集合
avgFitnessSet = zeros(maxGeneration, 1);                                    % 每代平均适应度集合

%% 进化
for i = 1 : maxGeneration
    rateOfProgress = 1 / maxGeneration;
    newPopulation = selectionOperationOfTournament(population, popFitness);	% 选择操作
    % [newPopulation] = crossoverOperationOfReal2(newPopulation, proC, disC);% 实数交叉操作
    % [newPopulation] = mutationOperationOfReal2(newPopulation, model.lower, model.upper, proM, disM);% 实数变异操作
    [newPopulation] = mutationOperationOfReal(newPopulation, mutationRate, model);
    newPopulation = repairOperation(newPopulation, model);          	    % 修复种群
    newPopFitness = getFitness(newPopulation, model);                       % 子代种群适应度
    [population, popFitness] = eliteStrategy(population, popFitness, newPopulation, newPopFitness, 2); % 精英策略
 
    
    [bestIndividual, bestFitness, avgFitness] = getBestIndividualAndFitness(population, popFitness);
    bestIndividualSet(i, :) = bestIndividual;                               % 第i代最优个体
    bestFitnessSet(i) = bestFitness;                                        % 第i代最高适应度
    avgFitnessSet(i) = avgFitness;                                          % 第i代种群平均适应度
    fprintf('第%i代种群的最优值：%.3f\n', i, -bestFitness);
   
    if mod(i, 1000) == 0                                                     % 每隔100代绘制一幅图，因为绘图代价较大
        close all;
        figure;
        model.showIndividual(bestIndividual, 0, model);                              % 路线可视化
        figure;
        showEvolCurve(10, i, -bestFitnessSet, -avgFitnessSet);              % 显示进化曲线
        % model.printResult(bestIndividual, model);
    end
end

[Cost1, Cost2, overDistanceOfDrone, numOfSamePoint] = model.getAllCost(bestIndividual, model);
fprintf('Cost%d: %.2f\n', problemId, Cost1);

model.printSubmitResult(bestIndividual, 0, model);
% bestFitnessSetGa = bestFitnessSet;
% save('.\result\bestFitnessSetGa.mat', 'bestFitnessSetGa');






