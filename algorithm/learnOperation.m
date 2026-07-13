function [newIndividual] = learnOperation(individual, goalIndividual, j)
    numOfDecVariables = length(individual);
    newIndividual = individual;
    
    G1 = individual(j);                                                     % 从个体0中获取第j个基因位上的基因gene
    index = find(goalIndividual == G1);                                      % 从个体1中找到基因gene所在位置
	index = index(randperm(length(index), 1));                              % 如果index为多个,随机取一个
    if index < numOfDecVariables                                            % 获取基因gene相邻位置
        index = index + 1;                                                  % 右侧的位置
    else
        index = index - 1;                                                  % 左侧的位置
    end
    G2 = goalIndividual(index);                                              % 与之前基因gene相邻的基因（注意此时的gene已被更新）
    index = find(individual == G2);                                         % 在第i个个体（待交叉个体）中找到基因gene所在位置
    rJ = index(randperm(length(index), 1));                                 % 如果index为多个,随机取一个
    if j < rJ
        newIndividual(j + 1: rJ) = individual(rJ: -1: j + 1);
    else
        newIndividual(rJ: j - 1) = individual(j - 1: -1: rJ);
    end
end

