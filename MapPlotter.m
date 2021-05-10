classdef MapPlotter
    properties
        HexSize {mustBeNumeric}
    end
    methods
        function obj = MapPlotter(val)
            if nargin == 1
                obj.HexSize = val;
            end
        end
    
        function drawAircraftPointingTo(obj, x, y, xTo, yTo)
            p1 = obj.getHexCenter(x, y);
            p2 = obj.getHexCenter(xTo, yTo);

            x = [p1(1), p2(1)];
            y = [p1(2), p2(2)];
            quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 0, 'LineWidth', 3);
        end

        function fillHex(obj, x, y, color)
            points = obj.getHexPoints(x, y);
            pgon = polyshape(points(:, 1), points(:, 2));
            ps = plot(pgon);
            ps.FaceColor = color;
        end

        function hexSize=hexSize(obj)
            hexSize = obj.HexSize;
        end

        function plotHexGrid(obj, width, height)
            for y = 0:height-1
                for x = 0:width-1
                    if mod(y, 2) == 0
                        points = obj.getHexPoints(x * 2, y);
                    else
                        points = obj.getHexPoints(x * 2 + 1, y);
                    end

                    plot(points(:, 1), points(:, 2), 'k');
                end
            end
        end

        function points=getHexPoints(obj, x, y)
            height = 2 * obj.hexSize();
            width = sqrt(3) * obj.hexSize();

            hexOffsets = [
                0, 0.5*height;
                0.5 * width, 0.25*height;
                0.5 * width, -0.25*height;
                0, -0.5*height;
                -0.5 * width, -0.25*height;
                -0.5 * width, 0.25*height;
                0, 0.5*height;
            ];

            center = obj.getHexCenter(x, y);

            points = [];

            for i=1:size(hexOffsets, 1)
                point = center + hexOffsets(i, :);
                points = [points; point];
            end
        end

        function pos=getHexCenter(obj, x, y)
            height = 2 * obj.hexSize();
            width = sqrt(3) * obj.hexSize();

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

            pos = [posX - 0.5*width + 1, posY + 1];
        end
    end
end