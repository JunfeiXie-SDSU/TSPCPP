% find intersection point of line y = mx + c (or x = c) and 
% line connecting points p1 and p2
function point = findInterOf2lines(m, c, p1, p2)
if abs(m*p1(1)+c-p1(2)) < 1e-8 
    if abs(m*p2(1)+c-p2(2)) < 1e-8 
        point = 0; % indicate both p1 and p2 are on line y
    else
        point = 1; % indicate the two lines intersect at p1
    end
elseif abs(m*p2(1)+c-p2(2))< 1e-8 
    point = 2; % indicate the two lines intersect at p2
else
    if p1(1) ~= p2(1)
        m2 = (p2(2) - p1(2))/(p2(1) - p1(1));
        c2 = (p1(2)*p2(1) - p2(2)*p1(1))/(p2(1) - p1(1));
        if m < inf
            point(1) = (c2-c)/(m-m2);
            point(2) = m2*point(1) + c2;
        else
            point(1) = c;
            point(2) = m2*point(1) + c2;
        end
        if sign(point(1) - p1(1))*sign(p2(1) - point(1)) ~= 1
            point = [];
        end

    else
        point(1) = p1(1);
        if m < inf
            point(2) = m*point(1) + c;
            if sign(point(2)-p1(2))*sign(p2(2)-point(2)) ~= 1
                point = [];
            end
        else
            point = [];
        end
    end
end

end