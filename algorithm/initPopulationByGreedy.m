function [population] = initPopulationByGreedy(populationSize, distanceMat, beta, greedyEpsilon, model)
    proOfVisitingMat = 1 ./ (distanceMat .^ beta);   
    
    individual = model.initIndividual(model);
    pointSet = sort(individual);
    numOfDecVariables = length(individual);
    
%     numOfDecVariables = size(distanceMat, 1);
%     pointSet = 1: numOfDecVariables;
    
    
    
    population = zeros(populationSize, numOfDecVariables);
    for i = 1 : populationSize
        startPoint = pointSet(randperm(numOfDecVariables, 1));
        if rand() < greedyEpsilon
            population(i, :) = getIndividualByGreedy(startPoint, pointSet, proOfVisitingMat, greedyEpsilon);
        else
            population(i, :) = model.initIndividual(model);
%             population(i, :) = randperm(numOfDecVariables);
        end
    end
end


function [individual] = getIndividualByGreedy(startPoint, pointSet, proOfVisitingMat, greedyEpsilon)
    numOfDecVariables = length(pointSet);
    individual = zeros(1, numOfDecVariables);
    individual(1) = startPoint;
    visiting = pointSet;
    
    index = find(pointSet == startPoint);
    index = index(1);        
    visiting(index) = [];                                                   % 待访问的点
    
    for i = 2 : numOfDecVariables
        point = individual(i - 1);                                          % 当前点
        proOfVisiting = proOfVisitingMat(point, visiting);                  % 待访问点的选择概率分布
        if rand() < greedyEpsilon
            [~, selectI] = max(proOfVisiting);
        else
            proOfVisiting = proOfVisiting / sum(proOfVisiting);
            proCum = cumsum(proOfVisiting);
            selectI = find(proCum >= rand(), 1);
            if isempty(selectI)
                selectI = randperm(length(visiting), 1);
            end
        end
        visited = visiting(selectI);                                         % 选中的点
        individual(i) = visited;
        visiting(selectI) = [];
    end
end

