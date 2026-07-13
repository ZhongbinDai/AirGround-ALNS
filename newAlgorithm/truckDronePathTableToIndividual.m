function [individual] = truckDronePathTableToIndividual(truckPathTable, realDronePathTableCell, model)
% 有问题，需要修改，存疑,可能还原，可能偏差
    dronePathTable = cell(model.numOfTruck, model.numOfDrone * model.numOfDroneMaxDepart);
    for truckId = 1: model.numOfTruck
        realDronePathTable = realDronePathTableCell{truckId};
        dronePathTable(truckId, :) = realDronePathTable(:);
        for d = 1: size(dronePathTable, 2)
            dPath = dronePathTable{truckId, d};
            if length(dPath) < 2
                dPath = [truckId truckId] + model.numOfCustomer;
            else
                dPath([1 end]) = [truckId truckId] + model.numOfCustomer;
            end
            dronePathTable{truckId, d} = dPath;
        end
    end
    pathTable = [truckPathTable dronePathTable];
    [individual] = model.getPathTableToIndividual(pathTable, model);
end

