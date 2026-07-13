function [population] = getPopulationOfAco(populationSize, numOfDecVariables, pheromoneMat, Eta, Alpha, Beta, model)
% 输入：种群规模、决策变量维度、信息素矩阵、启发因子、信息素重要程度参数、启发式因子重要程度参数
    population = zeros(populationSize, numOfDecVariables);
    individual = model.initIndividual(model);
    citySet = sort(individual);
    
    proOfVisitingMat = zeros(numOfDecVariables, numOfDecVariables);
    for i = 1 :numOfDecVariables
        I = citySet(i);
        for j = 1 : numOfDecVariables
            J = citySet(j);
            proOfVisitingMat(I, J) = (pheromoneMat(I, J) ^ Alpha) * (Eta(I, J) ^ Beta);
        end
    end
    

    for i = 1 : populationSize
        startCity = individual(mod(i, numOfDecVariables) + 1);
        population(i, :) = getIndividual(startCity, citySet, proOfVisitingMat);
    end
    
end

function [individual] = getIndividual(startCity, citySet, proOfVisitingMat)
    numOfDecVariables = length(citySet);
    individual = zeros(1, numOfDecVariables);
    individual(1) = startCity;
    
    index = find(citySet == startCity);
    index = index(1);
    visiting = [citySet(1:index - 1) citySet(index + 1 : end)];             % 待访问的城市
    
    for i = 2 : numOfDecVariables
        n = numOfDecVariables - i + 1;                                      % 待访问的城市数目
        proOfVisiting = zeros(1, n);                                        % 待访问城市的选择概率分布
        city = individual(i - 1);                                           % 当前城市
        for k = 1 : n                                                       % 计算待选城市的概率分布
            proOfVisiting(k) = proOfVisitingMat(city, visiting(k));
        end
        proOfVisiting = proOfVisiting / sum(proOfVisiting);
        % 按概率原则选取下一个城市
        proCum = cumsum(proOfVisiting);
        select = find(proCum >= rand, 1);                                   % 轮盘赌
        if isempty(select)
            select = randperm(n);
        end
        visited = visiting(select(1));                                      % 选中的城市
        individual(i) = visited;
        
        index = find(visiting == visited);
        index = index(1);                                                   % 城市可能重复
        visiting = [visiting(1:index - 1) visiting(index + 1 : end)];       % 剩余待访问的城市
    end

end





