clear; close all;

data = jsondecode(fileread("./results/ny_results.json"));
ny_map = imread('./data/ny_1_map.png');

width = data.size(1);
height = data.size(2);
hex_r_px = data.hex_r_px;

heights = data.heights;

obstacles = data.obstacles;


mp = MapPlotter(hex_r_px);


figure;

title('Testing map implementation');

% imagesc(heights);
imshow(ny_map);
hold on;
axis off;
% set(gca, 'YDir','reverse')

mp.plotHexGrid(width, height);

% for obstacleInd=1:size(obstacles, 1)
%     obstacle = obstacles(obstacleInd, :);
% 
%     mp.fillHex(obstacle(1), obstacle(2), 'r');
% end

xlim([-1 width*2*hex_r_px]);
ylim([-2 height*1.5*hex_r_px]);
