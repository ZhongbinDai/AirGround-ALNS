%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));                                                     % 将当前文件夹下的所有文件夹都包括进调用函数的目录
% rng('default');
problemId = 5;
numOfDroneMaxDepart = 5;
[model] = initModel3(problemId, numOfDroneMaxDepart);


pathTemp = solveTSP(model.coord);
figure;
model.drawPartPath(pathTemp, model);

% 
% ids = find(model.demandOfCustomer <= model.maxCapacityOfDrone);
% ids = [ids; size(model.coord, 1)];
% pathTemp = solveTSP(model.coord(ids, :));
% figure;
% model.drawPartPath(ids(pathTemp), model);