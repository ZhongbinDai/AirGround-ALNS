function [G, deg] = getDigraph(truckPathTable, realDronePathTableCell, model)
    timeMatOfTruck = model.timeMatOfTruck;
    timeMatOfDrone = model.timeMatOfDrone;

    n = size(timeMatOfTruck, 1);
    G = cell(n, 1);                                                         % 节点 p1 到 p2 的一条有向边，权重是对应交通方式的时间
    deg = zeros(n, 1);                                                      % 入度，便于后续拓扑排序

    for truckId = 1: numel(truckPathTable)
        truckPath = truckPathTable{truckId};
        if length(truckPath) > 2
            for i = 1: length(truckPath) - 1
                p1 = truckPath(i);
                p2 = truckPath(i + 1);
                G{p1} = [G{p1}; p2, timeMatOfTruck(p1, p2)];
				deg(p2) = deg(p2) + 1;
            end
        end
        realDronePathTable = realDronePathTableCell{truckId};
        for droneId = 1: size(realDronePathTable, 1)
            for departId = 1: size(realDronePathTable, 2)
                realDronePath = realDronePathTable{droneId, departId};
                if length(realDronePath) > 2
                    for i = 1: length(realDronePath) - 1
                        p1 = realDronePath(i);
                        p2 = realDronePath(i + 1);
                        G{p1} = [G{p1}; p2, timeMatOfDrone(p1, p2)];
				        deg(p2) = deg(p2) + 1;
                    end
                end
            end
        end
    end
end