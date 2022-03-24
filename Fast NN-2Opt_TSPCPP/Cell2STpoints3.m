% decompose a cell into grids 

function [path, b2] = Cell2STpoints3(vertices, movingD, l, b1)
gradient = vertices(end,1);
slope = vertices(end,2);
points = vertices(1:end-1,:);


if gradient == 0 
    height = max(points(:,1))-min(points(:,1));
    numGrids = ceil(height/l);
    delta_a2 = l/2;
    c1 = min(points(:,1));
    c2 = max(points(:,1));
elseif gradient == inf
    height = max(points(:,2))-min(points(:,2));
    numGrids = ceil(height/l);
    delta_a2 = l/2;
    c1 = min(points(:,2));
    c2 = max(points(:,2));
else % 0 < gradient < inf
    cs = points(:,2) - slope.*points(:,1);
    height = (max(cs) - min(cs))/sqrt(1+slope^2);
    numGrids = ceil(height/l);
    delta_a2 = sqrt(1+slope^2)*l/2;
    c1 = min(cs);
    c2 = max(cs);
end
%% find lines perpendiculr to support lines and the grid points lie on
if numGrids == 1 % only one grid
    b2 = (c1 + c2)/2;
else
    b2 = [c1+delta_a2;c2-delta_a2];
end
%% find points
numP = length(b2);
path = zeros(numP, 2);
for k = 1:numP
    if gradient == 0
        path(k,1) = b2(k);
        path(k,2) = b1;
    elseif gradient == inf
        path(k,1) = b1;
        path(k,2) = b2(k);
    else
        path(k,1) = (b2(k)-b1)/(gradient - slope);
        path(k,2) = (gradient*b2(k)-slope*b1)/(gradient - slope);
    end
end
if numP == 2
    vec = path(2,:)-path(1,:);
    mD = atan2(vec(2),vec(1));
    if mD*movingD > 0 
        path = path(2:-1:1,:);
    end
end

end