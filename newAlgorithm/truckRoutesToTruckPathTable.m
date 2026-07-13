function [truckPathTable, realDronePathTableCell] = truckRoutesToTruckPathTable(truckRoutes, droneRoutes, model)
    numOfCustomer = model.numOfCustomer;
    numOfTruck = model.numOfTruck;                                            % 卡车数量
    numOfDrone = model.numOfDrone;                                            % 每辆卡车的无人机数量
    numOfDroneMaxDepart = model.numOfDroneMaxDepart;
    
    truckPathTable = cell(numOfTruck, 1);
    realDronePathTableCell = cell(numOfTruck, 1);
    k = 0;
    for truckId = 1: numOfTruck
        truckPath = truckRoutes{truckId}';
        truckPath([1 end]) = numOfCustomer + truckId;
        truckPathTable{truckId} = truckPath;
    
        realDronePathTable = cell(numOfDrone, numOfDroneMaxDepart);
        for droneId = 1: numOfDrone
            k = k + 1;
            droneRouteArray = droneRoutes{k};
            for departId = 1: numel(droneRouteArray)
                dronePath = droneRouteArray{departId}';
                dronePath(1) = truckPath(dronePath(1) + 1);
                dronePath(end) = truckPath(dronePath(end) + 1);
                realDronePathTable{droneId, departId} = dronePath;
            end
        end 
        realDronePathTableCell{truckId} = realDronePathTable;
    end
end

