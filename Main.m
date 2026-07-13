%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng(0,'twister');
populationSize = 10;                                                        % 种群规模
maxGeneration = 2000;                                                       % 最大化迭代次数
numOfDroneMaxDepart = 5;                                                   % 每辆无人机最大出发次数

problemIds = 1: 10;
for i = 1: 10
    problemId = problemIds(i);                                              % 数据集Id
    [bestIndividualSet, bestFitnessSet, bestIndividual, bestFitness] = running1(problemId, numOfDroneMaxDepart, populationSize, maxGeneration);
end

