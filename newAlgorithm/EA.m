% -------------------------------------------------------------------------
% EA求解
% @作者：冰中呆
% @邮箱：1209805090@qq.com
% @时间：2025.10.17
% -------------------------------------------------------------------------
%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng('default');
problemId = 5;
numOfDroneMaxDepart = 10;
[model] = initModel3(problemId, numOfDroneMaxDepart);


maxGeneration = 2000;                                                       % 最大进化代数
%% 初始化
folderPath = sprintf('H:/提交记录/汇总分类/%02d/', problemId);
[population] = getPopulationByReadTxt(folderPath, model);
[population, ~, ~] = unique(population, 'rows', 'stable');
% populationSize = 100;
% [population] = initBestPopulation(populationSize, model);
populationSize = size(population, 1);                                       % 种群规模
population = repairOperation(population, model);                            % 修复种群
popFitness = getFitness(population, model);                                 % 计算种群适应度
numOfDecVariables = size(population, 2);                                    % 决策变量维度

bestIndividualSet = zeros(maxGeneration, numOfDecVariables);                % 每代最优个体集合
bestFitnessSet = zeros(maxGeneration, 1);                                   % 每代最高适应度集合
avgFitnessSet = zeros(maxGeneration, 1);                                    % 每代平均适应度集合

for t = 1: maxGeneration
    [newPopulation] = perturbationOperation(population, model);
    newPopulation = repairOperation(newPopulation, model);          	    % 修复种群
    % [newPopulation] = repairOperation2(newPopulation, model);
    newPopFitness = getFitness(newPopulation, model);                       % 计算种群适应度
    [population, popFitness] = eliteStrategy(population, popFitness, newPopulation, newPopFitness, 1);

    [bestIndividual, bestFitness, avgFitness] = getBestIndividualAndFitness(population, popFitness);
    avgFitnessSet(t) = avgFitness;       
    bestFitnessSet(t) = bestFitness;
    bestIndividualSet(t, :) = bestIndividual;
    fprintf('第%i代种群的最优值：%.6f\n', t, -bestFitness);
    if mod(t, 50) == 0
        model.printIndividual(bestIndividual, model);
        model.printSubmitResult(bestIndividual, 1, model);
        model.saveResult(bestIndividual, model);

        dateString = datestr(now, 'yyyymmdd-HHMM');
        filePath = sprintf('./result/mat/problem%02d-%s.mat', problemId, dateString);
        ensureDirectory(filePath)
        save(filePath);

        close all; 
        figure;
        model.showIndividual(bestIndividual, 0, model);
        figure;
        showEvolCurve(1, t - 1, -bestFitnessSet, -avgFitnessSet);           % 显示进化曲线
    end
end


bestFitnessSetEa = bestFitnessSet;
save('./result/bestFitnessSetEa.mat', 'bestFitnessSetEa');



