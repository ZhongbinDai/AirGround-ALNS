function [population] = initBestPopulation(populationSize, model)
    beta = 2;                                                                   % 贪婪因子重要程度
    greedyEpsilon = 1;                                                          % 贪婪度

    distanceMat = model.distanceMatOfDrone;
    numOfDecVariables = model.numOfDecVariables;                                % 决策变量维度
    Eta = 1 ./ distanceMat;                                                     % Eta为启发因子,这里设为距离的倒数
    pheromoneMat = ones(numOfDecVariables, numOfDecVariables);                 	% pheromone为信息素矩阵
    population1 = initPopulationByGreedy(populationSize, distanceMat, beta, greedyEpsilon, model);
    population2 = getPopulationOfAco(populationSize, numOfDecVariables, pheromoneMat, Eta, 1, 5, model);
    population3 = initialPopulation(populationSize, model);                      % 初始化种群
    
    population1 = repairOperation(population1, model);   
    population2 = repairOperation(population2, model);   
    population3 = repairOperation(population3, model);

    popFitness1 = getFitness(population1, model);
    popFitness2 = getFitness(population2, model);
    popFitness3 = getFitness(population3, model);

    [population, popFitness] = eliteStrategy(population1, popFitness1, population2, popFitness2, 2); % 精英策略
    [population, popFitness] = eliteStrategy(population, popFitness, population3, popFitness3, 2);

    for randTruck = 0.1: 0.1: 1
        [newPopulation] = initPopulationByGreedy2(populationSize, beta, greedyEpsilon, randTruck, model);
        [newPopulation] = repairOperation(newPopulation, model);                % 修复种群，防止越界
        newPopFitness = getFitness(newPopulation, model);                       % 子代种群适应度
        [population, popFitness] = eliteStrategy(population, popFitness, newPopulation, newPopFitness, 2); % 精英策略
    end
end

