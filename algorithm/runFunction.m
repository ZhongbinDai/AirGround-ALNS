function [bestIndividualSet, bestFitnessSet, bestIndividual, bestFitness] = runFunction(problemId, populationSize, maxGeneration)
    fileName = sprintf('./data/%d.txt', problemId);
    [model] = initModel1(fileName);
    
    crossoverRate = 0.6;                                                        % 交叉概率
    mutationRate = 0.01;                                                        % 变异概率
    T = 100;                                                                    % 初始温度
    AttenuationRate = 0.99;                                                     % 温度衰减系数
    lambdaRate = 0.5;                                                           % 算子权重的挥发系数
    numOfDestroy = round(model.numOfDecVariables * 0.02);                        % 破坏的节点数量
    % numOfDestroy = 2;
    weightsOfDestroy = ones(1,3);                                               % destroy算子的初始权重
    weightsOfRepair = ones(1,2);                                                % repair算子的初始权重
    cntOfDestroy = zeros(1,3);                                                  % destroy算子的使用次数
    cntOfRepair = zeros(1,2);                                                   % repair算子的使用次数
    scoreOfDestroy = zeros(1,3);                                                % destroy算子的得分
    scoreOfRepair = zeros(1,2);                                                 % repair算子的得分
    scoreSet = [0.5 0.8 1.2 1.5];
    
    beta = 2;                                                                   % 贪婪因子重要程度
    greedyEpsilon = 1;                                                          % 贪婪度
    %% 初始化
    distanceMat = model.distanceMatOfDrone;
    numOfDecVariables = model.numOfDecVariables;                                % 决策变量维度
    Eta = 1 ./ distanceMat;                                                     % Eta为启发因子,这里设为距离的倒数
    pheromoneMat = ones(numOfDecVariables, numOfDecVariables);                 	% pheromone为信息素矩阵
    % [population] = initPopulationByGreedy(populationSize, distanceMat, beta, greedyEpsilon, model);
    % population = initialPopulation(populationSize, model);                      % 初始化种群
    % population = getPopulationOfAco(populationSize, numOfDecVariables, pheromoneMat, Eta, 1, 5, model);
    [population] = initBestPopulation(populationSize, model);
    population = repairOperation(population, model);                            % 修复种群
    popFitness = getFitness(population, model);                                 % 计算种群适应度

    [bestFitness, I] = max(popFitness);
    bestIndividual = population(I, :);
    
    bestIndividualSet = zeros(maxGeneration, model.numOfDecVariables);          % 每代最优个体集合
    bestFitnessSet = zeros(maxGeneration, 1);                                   % 每代最高适应度集合
    currentIndividualSet = zeros(maxGeneration, model.numOfDecVariables);       % 每代当前个体集合
    currentFitnessSet = zeros(maxGeneration, 1);                                % 每代当前适应度集合
    avgFitnessSet = zeros(maxGeneration, 1);                                    % 每代平均适应度集合
    %% 进化
    for t = 1: maxGeneration
        
        [popDestroyOperatorIds, popRepairOperatorIds] = getPopulationDestroyRepairOperatorIds(populationSize, weightsOfDestroy, weightsOfRepair);
        [cntOfDestroy, cntOfRepair] = updateCntOfDestroyRepair(cntOfDestroy, cntOfRepair, popDestroyOperatorIds, popRepairOperatorIds);
        
        [popPart, popDestroyPart] = destroyPopOperationOfAlns(population, numOfDestroy, popDestroyOperatorIds, model);
        [newPopulation] = repairPopOperationOfAlns(popPart, popDestroyPart, popRepairOperatorIds, model);
        newPopulation = repairOperation(newPopulation, model);                  % 修复种群
        newPopFitness = getFitness(newPopulation, model);                       % 子代种群适应度
    
        [bestIndividual, bestFitness, avgFitness] = getBestIndividualAndFitness([population; newPopulation], [popFitness; newPopFitness]);
        [weightsOfDestroy, weightsOfRepair, population, popFitness] = updateWeightOfDestroyRepair(weightsOfDestroy, weightsOfRepair, population, popFitness, newPopulation, newPopFitness, bestFitness, popDestroyOperatorIds, popRepairOperatorIds, scoreSet, cntOfDestroy, cntOfRepair, lambdaRate, T);
        population(1, :) = bestIndividual;
        popFitness(1) = bestFitness;
    
        newPopulation = selectionOperationOfTournament(population, popFitness);	% 选择操作
	    [newPopulation] = crossoverOperationOfTsp(newPopulation, crossoverRate);% 交叉操作
        [newPopulation] = mutationOperationOfTsp(newPopulation, mutationRate);  % 变异操作
        [newPopulation] = repairOperation(newPopulation, model);                % 修复种群，防止越界
        newPopFitness = getFitness(newPopulation, model);                       % 子代种群适应度
        [population, popFitness] = eliteStrategy(population, popFitness, newPopulation, newPopFitness, 2); % 精英策略
        [bestIndividual, bestFitness, avgFitness] = getBestIndividualAndFitness(population, popFitness);
        
        bestIndividualSet(t, :) = bestIndividual;                               % 每代最优个体集合
        bestFitnessSet(t) = bestFitness;                                        % 每代最高适应度集合
        avgFitnessSet(t) = mean(popFitness);                                    % 每代当前适应度集合
        
	    fprintf('第%d代种群的最优值：%f\n', t, - bestFitness);
        
        if mod(t, 100) == 0                                                    % 每隔100代绘制一幅图，因为绘图代价较大
            close all; 
            figure;
            showIndividual(bestIndividual, model);                              % 路线可视化
            figure;
            showEvolCurve(1, t - 1, -bestFitnessSet, -avgFitnessSet);             % 显示进化曲线
            model.printIndividual(bestIndividual, model);
            model.printIndividualOfCompetition(bestIndividual, model);
            
            fileName = sprintf('./saveResult/Instance%d.txt', problemId);
            model.saveIndividualOfCompetition(bestIndividual, fileName, model);
            dateString = datestr(now, 'yyyy-mm-dd');
            saveFileName = sprintf('./result/result%02d-%s.mat', problemId, dateString);
            save(saveFileName);
        end
        T = T * AttenuationRate;
    end
    
end

