%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));

% 计算每日提交结果值
for problemId = 1: 10
    numOfDroneMaxDepart = 10;
    [model] = initModel3(problemId, numOfDroneMaxDepart);
    fileName = sprintf('H:/提交记录/saveResult-2025-10-20 下/Instance%d.txt', problemId);
    [truckRoutes, droneRoutes] = readTruckDroneRoutes(fileName);
    [truckPathTable, realDronePathTableCell] = truckRoutesToTruckPathTable(truckRoutes, droneRoutes, model);
    [Cost] = model.getCost3(truckPathTable, realDronePathTableCell, model);

    % model.showIndividual2(truckPathTable, realDronePathTableCell, 0, model);
    model.printIndividual2(truckPathTable, realDronePathTableCell, model);
    % fprintf('%.6f\n', Cost);
end