%% 清空
clear;                                                                      % 清除所有变量
close all;                                                                  % 清图
clc;                                                                        % 清屏
%% 参数配置
addpath(genpath('.\'));

% 测试用例1：简单的4个点
coordOfPoint = rand(30, 2);
path1 = solveTSP(coordOfPoint);
% fprintf('坐标矩阵:\n');
% disp(coordOfPoint);
% fprintf('最优路径: %s\n', mat2str(path1));


figure;
hold on;

xCoord = coordOfPoint(:, 1);
yCoord = coordOfPoint(:, 2);
plot(xCoord(path1), yCoord(path1), '-o', 'LineWidth', 1);
