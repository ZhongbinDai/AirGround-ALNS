%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));

problemId = 10;
numOfDroneMaxDepart = 10;
[model] = initModel3(problemId, numOfDroneMaxDepart);
% figure;
% hold on;
% grid on;
% axis equal;
% model.drawPoints(model);


ids = [112 80 62 156 132 64 183];
coordOfPoints = model.coordOfCustomer(ids, :);
xCoord = coordOfPoints(:, 1);
yCoord = coordOfPoints(:, 2);


figure;
hold on;
grid on;
axis equal;
plot(xCoord, yCoord, 'ob', 'MarkerSize', 5,'LineWidth', 1, 'HandleVisibility', 'off');
path1 = [1 2 3 4 5];
path2 = [1 6 7 5];
plot(xCoord(path1), yCoord(path1), '-', 'LineWidth', 1, 'DisplayName', '车辆');
plot(xCoord(path2), yCoord(path2), '--', 'LineWidth', 1, 'DisplayName', '无人机');
legend('Location', 'northeastoutside');



figure;
hold on;
grid on;
axis equal;
plot(xCoord, yCoord, 'ob', 'MarkerSize', 5,'LineWidth', 1, 'HandleVisibility', 'off');
path1 = [1 2 3 4 5];
path2 = [1 7 6 5];
plot(xCoord(path1), yCoord(path1), '-', 'LineWidth', 1, 'DisplayName', '车辆');
plot(xCoord(path2), yCoord(path2), '--', 'LineWidth', 1, 'DisplayName', '无人机');
legend('Location', 'northeastoutside');