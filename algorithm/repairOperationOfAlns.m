function [individual] = repairOperationOfAlns(individualPart, destroyPart, operatorId, model)
    if operatorId == 1
        [individual] = insertByRandom(individualPart, destroyPart);
    else
        [individual] = insertByGreedy(individualPart, destroyPart, model);
    end
end

function [individual] = insertByRandom(individualPart, destroyPart)
    individual = individualPart;
    for i = 1: length(destroyPart)
        r = randi(length(individual) + 1);
        individual = [individual(1: r - 1) destroyPart(i) individual(r: end)];
    end
end


function [individual] = insertByGreedy(individualPart, destroyPart, model)
    individual = individualPart;
    for i = 1: length(destroyPart)
        maxValue = - inf;
        insertIndex = - 1;
        
        individualTempTable = zeros(length(individual) + 1, length(individual) + 1);
        individualTempFitArray = zeros(length(individual) + 1, 1) - inf;
        n = 0;
        % 这块开销非常大，可优化
        for j = 1: length(individual) + 1
            individualTemp = [individual(1: j - 1) destroyPart(i) individual(j: end)];
            individualTemp = model.repairIndividual(individualTemp, model);
            % [individualTempFitness] = model.getIndividualFitness(individualTemp, model);
            [individualTempFitness, individualTempTable, individualTempFitArray, n] = getIndividualTempFitness(individualTemp, individualTempTable, individualTempFitArray, n, model);

            if maxValue < individualTempFitness
                maxValue = individualTempFitness;
                insertIndex = j;
            end
        end
        individual = [individual(1: insertIndex - 1) destroyPart(i) individual(insertIndex: end)];
        % fprintf('重复率:%.4f\n', 1 - n / length(individualTempFitArray));
    end
end


function [individualTempFitness, individualTempTable, individualTempFitArray, n] = getIndividualTempFitness(individualTemp, individualTempTable, individualTempFitArray, n, model)
    [rowIdx, ~] = find(ismember(individualTempTable, individualTemp, 'rows'), 1);
    if isempty(rowIdx)
        % disp('未找到目标行');
        [individualTempFitness] = model.getIndividualFitness(individualTemp, model);
        n = n + 1;
        individualTempTable(n, :) = individualTemp;
        individualTempFitArray(n) = individualTempFitness;
    else
        % disp('*找到目标行');
        individualTempFitness = individualTempFitArray(rowIdx);
    end
end

