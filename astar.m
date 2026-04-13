function path = astar(map, start, goal)

    [maxX, maxY] = size(map);

    gScore = inf(maxX, maxY); %stores the best cost from start cell to current cell
    fScore = inf(maxX, maxY); %stores f = g + h
    cameFrom = zeros(maxX, maxY, 2); %stores the parent's of each node, helps in reconstruction of the final path

    openSet = false(maxX, maxY);
    closedSet = false(maxX, maxY);

    gScore(start(1), start(2)) = 0; %at start the cost to current cell is 0
    fScore(start(1), start(2)) = heuristic(start, goal); %total cost is only dependent on h
    openSet(start(1), start(2)) = true; %start is the first node in the open cell

    %this is right,left,up,down (4-connected motion)   
    directions = [ 1  0;
                  -1  0;
                   0  1;
                   0 -1 ];

    while any(openSet(:))

        %finds all open nodes and chooses the path with least f
        openIndices = find(openSet);
        [~, idxMin] = min(fScore(openIndices));
        currentLinear = openIndices(idxMin);
        [cx, cy] = ind2sub(size(openSet), currentLinear);

        current = [cx, cy];

        if isequal(current, goal)
            path = reconstructPath(cameFrom, current, start);
            return;
        end

        openSet(cx, cy) = false;
        closedSet(cx, cy) = true;

        for k = 1:size(directions,1)
            neighbor = current + directions(k,:);
            nx = neighbor(1);
            ny = neighbor(2);

            if nx < 1 || nx > maxX || ny < 1 || ny > maxY
                continue;
            end

            if map(nx, ny) == 1
                continue;
            end

            if closedSet(nx, ny)
                continue;
            end

            tentativeG = gScore(cx, cy) + 1;

            %if neighbor never considered before, or current path is...
            %... better than old one, it updates parents and costs
            if ~openSet(nx, ny) || tentativeG < gScore(nx, ny)
                cameFrom(nx, ny, :) = current;
                gScore(nx, ny) = tentativeG;
                fScore(nx, ny) = tentativeG + heuristic([nx, ny], goal);
                openSet(nx, ny) = true;
            end
        end
    end

    path = [];
end

function h = heuristic(node, goal)
    h = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function path = reconstructPath(cameFrom, current, start)
    path = current;

    while ~isequal(current, start)
        parent = squeeze(cameFrom(current(1), current(2), :))';
        current = parent;
        path = [current; path];
    end
end

%[appendix]{"version":"1.0"}
%---
