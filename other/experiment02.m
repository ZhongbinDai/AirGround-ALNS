%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng(0,'twister');
populationSize = 100;
maxGeneration = 2000;
numOfDroneMaxDepart = 10;

problemIds = [8 10 9 1: 7];
problemIds = 3: 10;
for i = 1: 10
    problemId = problemIds(i);
    [bestIndividualSet, bestFitnessSet, bestIndividual, bestFitness] = running2(problemId, numOfDroneMaxDepart, populationSize, maxGeneration);
end