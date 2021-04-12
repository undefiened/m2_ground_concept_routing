clear; close all;

res = rmdir("./results/images/", 's');
display(res);
mkdir("./results/images/");

data = jsondecode(fileread("./results/results.json"));

width = data.map.width;
height = data.map.height;

flightplans = data.flightplans;
obstacles = data.obstacles;

minTime = +Inf;
maxTime = -Inf;

for i=1:size(flightplans, 1)
    flightplan = flightplans{i};
    if max(flightplan(:, 1)) > maxTime
        maxTime = max(flightplan(:, 1));
    end
    
    if min(flightplan(:, 1)) < minTime
        minTime = min(flightplan(:, 1));
    end
end

for time=minTime:maxTime
    figure;
    hold on;
    title(['Time ' num2str(time)]);

    plotHexGrid(width, height);
    
    for obstacleInd=1:size(obstacles, 1)
        obstacle = obstacles(obstacleInd, :);
        
        fillHex(obstacle(1), obstacle(2), 'r');
    end

    for flightplanId=1:size(flightplans, 1)
%         flightplan = squeeze(flightplans(flightplanId, :, :));
        flightplan = flightplans{flightplanId};
        
        pointInd = find(flightplan(:, 1) == time);
        
        if length(pointInd) == 1
            point = flightplan(pointInd, 2:3);
            
            fillHex(point(1), point(2), 'b');
            
            if pointInd < size(flightplan, 1)
                nextPoint = flightplan(pointInd+1, 2:3);
                drawAircraftPointingTo(point(1), point(2), nextPoint(1), nextPoint(2))
            end
        end
    end
    xlim([-1 width*2]);
    ylim([-2 height*1.5]);
    saveas(gcf, ['./results/images/res_' num2str(time, '%03.f') '.png']);
    close all;
end



function drawAircraftPointingTo(x, y, xTo, yTo)
    p1 = getHexCenter(x, y);
    p2 = getHexCenter(xTo, yTo);
    
    x = [p1(1), p2(1)];
    y = [p1(2), p2(2)];
    quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 0, 'LineWidth', 3);
end

function fillHex(x, y, color)
    points = getHexPoints(x, y);
    pgon = polyshape(points(:, 1), points(:, 2));
    ps = plot(pgon);
    ps.FaceColor = color;
end

function hexSize=hexSize()
    hexSize = 1;
end

function plotHexGrid(width, height)
    for y = 0:height-1
        for x = 0:width-1
            if mod(y, 2) == 0
                points = getHexPoints(x * 2, y);
            else
                points = getHexPoints(x * 2 + 1, y);
            end

            plot(points(:, 1), points(:, 2), 'k');
        end
    end
end

function points=getHexPoints(x, y)
    height = 2 * hexSize();
    width = sqrt(3) * hexSize();
    
    hexOffsets = [
        0, 0.5*height;
        0.5 * width, 0.25*height;
        0.5 * width, -0.25*height;
        0, -0.5*height;
        -0.5 * width, -0.25*height;
        -0.5 * width, 0.25*height;
        0, 0.5*height;
    ];
    
    center = getHexCenter(x, y);
    
    points = [];
    
    for i=1:size(hexOffsets, 1)
        point = center + hexOffsets(i, :);
        points = [points; point];
    end
end

function pos=getHexCenter(x, y)
    height = 2 * hexSize();
    width = sqrt(3) * hexSize();
    
    if mod(x, 2) == 0 && mod(y, 2) == 0
        posX = (x/2 + 0.5)*width;
        posY = 0.75*y*height;
    elseif mod(x, 2) ~= 0 && mod(y, 2) ~= 0
        posX = ((x-1)/2 + 1)*width;
        posY = 0.75*y*height;
    else
        ME = MException('PlotResultsL:WrongHexCoordinates', ...
        'Hex coordinates %s are not consistent', [x, y]);
        throw(ME);
    end
    
    pos = [posX, posY];
end