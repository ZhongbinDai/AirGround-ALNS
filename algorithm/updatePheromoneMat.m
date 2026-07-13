function [pheromone] = updatePheromoneMat(pheromone, population, popFitness, Rho, Q)
% 更新信息素
    populationSize = size(population, 1);
    numOfDecVariables = size(population, 2);
    Delta_Tau=zeros(numOfDecVariables,numOfDecVariables);
    for i=1:populationSize
        for j=1:(numOfDecVariables-1)
            Delta_Tau(population(i,j),population(i,j+1))=Delta_Tau(population(i,j),population(i,j+1))+Q/popFitness(i);
        end
        Delta_Tau(population(i,numOfDecVariables),population(i,1))=Delta_Tau(population(i,numOfDecVariables),population(i,1))+Q/popFitness(i);
    end
    pheromone=(1-Rho).*pheromone+Delta_Tau;
    
end

