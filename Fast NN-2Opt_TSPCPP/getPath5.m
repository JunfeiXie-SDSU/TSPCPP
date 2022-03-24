% given an edge (a,b), compute a back and forth path with origin at vertex
% a with sweep direction perpendicular to (a,b), a and b are indices
% Difference from getPath() is: starts from internal grids
% Difference from getPath2(): w_grid of the first and last cell is r

function [path, cost] = getPath5(x, y, a, b, UAS_range)

l = UAS_range(1);
w = UAS_range(2);
if b-a == 1 
    [Width, ~, anti_indx] = SpanOfConvexPolygon(x,y,a); %counter-clockwise
elseif a-b ~= 1 && b == 1
    [Width, ~, anti_indx] = SpanOfConvexPolygon(x,y,a); %counter-clockwise
else
    [Width, ~, anti_indx] = SpanOfConvexPolygon(x,y,b); 
end

if Width/w - floor(Width/w) > 1e-5
    numLines = ceil(Width/w)+1;
else
    numLines = floor(Width/w)+1;
end
if numLines > 2 
    w_b = (Width - w)/(numLines - 2); % width of the middle bars
end

% find the sweep lines
edge = [x(a), y(a); x(b), y(b)];
vec = edge(2,:)-edge(1,:);
movingD = atan2(vec(2),vec(1));
if edge(1,1) ~= edge(2,1)
    gradient = (edge(2,2)-edge(1,2))/(edge(2,1)-edge(1,1));
    if numLines > 2
        delta_b = sqrt(1+gradient^2)*w_b;
        delta_ab = sqrt(1+gradient^2)*(w_b+w)/2;
    end
    
    c1 = edge(1,2) - gradient*edge(1,1);
    c2 = y(anti_indx) - gradient*x(anti_indx);
    % slope of the line perpendicular to support lines
    if gradient ~= 0
        slope = -1/gradient;
    else
        slope = inf;
    end
else
    gradient = inf;
    c1 = edge(1,1);
    c2 = x(anti_indx);
    if numLines > 2
        delta_b = w_b;
        delta_ab = (w_b+w)/2;
    end
    slope = 0;
end

if c1 < c2 
    if numLines == 2
        support_c = [c1, c2];
    elseif numLines == 3
        support_c = [c1,c1+delta_ab,c2];
    else
        support_c = [c1,c1+delta_ab:delta_b:c2-delta_ab,c2];
        if length(support_c) < numLines
            support_c = [support_c(1:end-1), c2-delta_ab, c2];
        end
    end
else
    if numLines == 2
        support_c = [c1, c2];
    elseif numLines == 3
        support_c = [c1,c1-delta_ab,c2];
    else
        support_c = [c1,c1-delta_ab:-delta_b:c2+delta_ab,c2];
        if length(support_c) < numLines
            support_c = [support_c(1:end-1), c2+delta_ab, c2];
        end
    end
end

% decompose the region between two adjacent sweep lines into cells
path = cell(2,1);
path{1}=[];
path{2}=[];
Cells = cell(numLines-1, 1);
for i = 1:numLines-1
    Cells{i} = [];
end
x_temp = x;
y_temp = y;
if length(support_c) <= 2 % only one cell
    % no need to further decompose
    Cells{1} = [x, y; gradient, slope];
    dis = (support_c(1) + support_c(2))/2;
    points = Cell2STpoints3(Cells{1}, movingD, l, dis);
    path{1} = [path{1}; points];
    if length(points(:,1)) > 1
        path{2} = [path{2}; points(2,:); points(1,:)];
    else
        path{2} = [path{2}; points];
    end
else
    for i = 1:length(support_c)-2
        c1 = support_c(i);
        c2 = support_c(i+1);
        % separate the original region into two sub-regions
        [sR, lR] = SeparateRegion(x_temp,y_temp,gradient,c1,c2);
        x_temp = lR(:,1);
        y_temp = lR(:,2);
        % the last two rows of Cells save the info of the support lines
        Cells{i} = [sR; gradient, slope]; 
        if ~isempty(path{1}) 
            vec = -vec;
            movingD = atan2(vec(2),vec(1));
        end
        if i == 1
            if c1 < c2 
                if gradient < inf
                    dis = c1 + sqrt(1+gradient^2)*w/2;
                else
                    dis = c1 + w/2;
                end
            else
                if gradient < inf  
                    dis = c1 - sqrt(1+gradient^2)*w/2;
                else
                    dis = c1 - w/2;
                end
            end
        else
            dis = (c1 + c2)/2;
        end
        points = Cell2STpoints3(Cells{i}, movingD, l, dis);
        path{1} = [path{1}; points];
        if length(points(:,1)) > 1
            path{2} = [path{2}; points(2,:); points(1,:)];
        else
            path{2} = [path{2}; points];
        end
    end
    c1 = support_c(i+1); % last cell
    c2 = support_c(i+2);
    Cells{i+1} = [lR; gradient, slope];
    vec = -vec;
    movingD = atan2(vec(2),vec(1));
    if c1 < c2
        if gradient < inf
            dis = c2 - sqrt(1+gradient^2)*w/2;
        else
            dis = c2 - w/2;
        end
    else
        if gradient < inf
            dis = c2 + sqrt(1+gradient^2)*w/2;
        else
            dis = c2 + w/2;
        end
    end
    points = Cell2STpoints3(Cells{i+1}, movingD, l, dis);
    path{1} = [path{1}; points];
    if length(points(:,1)) > 1
        path{2} = [path{2}; points(2,:); points(1,:)];
    else
        path{2} = [path{2}; points];
    end
end

cost = zeros(2,1);
for k = 1:2
    for i = 1:size(path{k},1)-1
        if i >= 2
            cost(k) = cost(k) + norm(path{k}(i,:)-path{k}(i+1,:));
        else
            cost(k) = cost(k) + norm(path{k}(i,:)-path{k}(i+1,:));
        end
    end
end
