%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng(0,'twister');
populationSize = 5;
maxGeneration = 10;
problemId = 2;

parfor problemId = 1: 10
    [bestIndividualSet, bestFitnessSet, bestIndividual, bestFitness] = runFunction(problemId, populationSize, maxGeneration);
end