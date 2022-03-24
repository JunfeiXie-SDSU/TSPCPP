% separte a region into two subregions 
function [sR, lR] = SeparateRegion(x,y,m,c1,c2)
numVert = length(x);
InterIndx = [];
P = [];
for j = 1:numVert
    p1 = [x(j), y(j)];
    if j < numVert
        next = j+1;
    else
        next = 1;
    end
    p2 = [x(next), y(next)];
    point = findInterOf2lines(m, c2, p1, p2);
    if ~isempty(point) % what if the line intersects at the vertex???
        if length(point) == 2
            InterIndx = [InterIndx; j, next];
            P = [P; point];
        elseif length(point) == 1
            if point == 1
                InterIndx = [InterIndx; j, 0];
            elseif point == 2
                InterIndx = [InterIndx; next, 0];
            end
        end
    end
end

% must have two intersection points
if size(InterIndx,1) == 2
    in1 = [InterIndx(1,1), InterIndx(2,2)];
    in2 = [InterIndx(1,2), InterIndx(2,1)];
    set1 = [x(1:in1(1)), y(1:in1(1)); P];
    if in1(2)> 1
        set1 = [set1; x(in1(2):end), y(in1(2):end)];
    end

    set2 = [P(end:-1:1,:); x(in2(1):in2(2)), y(in2(1):in2(2))];
elseif size(InterIndx, 1) == 3 % one intersection point is a vertex
    r1 = find(InterIndx(:,2) ~= 0);
    r2 = find(InterIndx(:,2) == 0);
    if r1 == 1
        set1 = [x(1:InterIndx(r1,1)), y(1:InterIndx(r1,1));P; ...
            x(InterIndx(r2(1),1):end), y(InterIndx(r2(1),1):end)];
        set2 = [P; x(InterIndx(r1,2):InterIndx(r2(1),1)), y(InterIndx(r1,2):InterIndx(r2(1),1))];
    else
        set1 = [x(1:InterIndx(r1(1),1)), y(1:InterIndx(r1,1));P; ...
            x(InterIndx(r2,2):end), y(InterIndx(r2,2):end)];
        set2 = [x(InterIndx(r1(1),1):InterIndx(r2,1)), y(InterIndx(r1(1),1):InterIndx(r2,1));P];
    end
    
elseif size(InterIndx, 1) == 4 % both intersection points are vertice
    r1 = sort(unique(InterIndx(:,1)));
    set1 = [x(1:r1(1)), y(1:r1(1)); x(r1(2):end), y(r1(2):end)];
    set2 = [x(r1(1):r1(2)), y(r1(1):r1(2))];
else
    error('no intersection points');
end

indicator = Between2Lines(set2(end,1),set2(end,2), m,c1,c2);
if indicator == 0
    sR = set1;
    lR = set2;
else
    sR = set2;
    lR = set1;
end
end