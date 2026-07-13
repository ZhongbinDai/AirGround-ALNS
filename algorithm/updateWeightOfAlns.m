function [newWeight] = updateWeightOfAlns(weight, score, cnt, lambdaRate)
    newWeight = weight * lambdaRate + (1 - lambdaRate) * score / cnt;
end

