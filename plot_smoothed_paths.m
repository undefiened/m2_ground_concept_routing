clear; close all;

data = jsondecode(fileread("./results/ny_results.json"));
ny_map = imread('./data/ny_1_map.png');

width = data.map.width;
height = data.map.height;
hex_r_px = 1;

obstacles = data.obstacles;

flightplans = data.flightplans;

if ~iscell(flightplans)
    newFlightplans = {};
    for i=1:size(flightplans, 1)
        newFlightplans{i, 1} = squeeze(flightplans(i, :, :));
    end
    flightplans = newFlightplans;
end

smoothedFlightplans = data.smoothed_flightplans;
if ~iscell(smoothedFlightplans)
    newFlightplans = {};
    for i=1:size(smoothedFlightplans, 1)
        newFlightplans{i, 1} = squeeze(smoothedFlightplans(i, :, :));
    end
    smoothedFlightplans = newFlightplans;
end


minTime = +Inf;
maxTime = -Inf;
totalTime = 0;

for i=1:size(flightplans, 1)
    flightplan = flightplans{i};
    if max(flightplan(:, 1)) > maxTime
        maxTime = max(flightplan(:, 1));
    end
    
    if min(flightplan(:, 1)) < minTime
        minTime = min(flightplan(:, 1));
    end
    
    totalTime = totalTime + size(flightplan, 1);
end

mp = MapPlotter(hex_r_px);


figure;
hold on;
title('Testing map implementation');


mp.plotHexGrid(width, height);

for obstacleInd=1:size(obstacles, 1)
    obstacle = obstacles(obstacleInd, :);

    mp.fillHex(obstacle(1), obstacle(2), 'r');
end

for i=1:size(flightplans, 1)
    flightplan = flightplans{i};
    smoothedFlightplan = smoothedFlightplans{i};
    
    pointsEuclidean = [];
    smoothedPointsEuclidean = [];
    
    for point=1:size(flightplan, 1)
        pointsEuclidean = [pointsEuclidean; mp.getHexCenter(flightplan(point, 2), flightplan(point, 3))];
    end
    
    for point=1:size(smoothedFlightplan, 1)
        smoothedPointsEuclidean = [smoothedPointsEuclidean; mp.getHexCenter(smoothedFlightplan(point, 2), smoothedFlightplan(point, 3))];
    end
    
    plot(pointsEuclidean(:, 1), pointsEuclidean(:, 2), 'b');
    plot(smoothedPointsEuclidean(:, 1), smoothedPointsEuclidean(:, 2), 'r');
end



xlim([-1 width*2*hex_r_px]);
ylim([-2 height*1.5*hex_r_px]);

saveas(gcf, ['./results/images/smoothed_paths.png']);