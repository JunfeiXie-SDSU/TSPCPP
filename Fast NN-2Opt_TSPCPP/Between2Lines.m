%%% see whether a point is between two lines mx+c1 and mx+c2 
function indicator = Between2Lines(x,y,m,c1,c2)
indicator = 0;
if m < inf
    temp1 = y - (m*x + c1);
    temp2 = y - (m*x + c2);
else
    temp1 = x - c1;
    temp2 = x - c2;
end

if abs(temp1) < 1e-8
    temp1 = 0;
end

if abs(temp2) < 1e-8
    temp2 = 0;
end

if c1 < c2
    % point between the two lines meet the criteria
    % y-(mx+c1) >= 0 and y-(mx+c2) =< 0, vice versa
    if temp1 >= 0 && temp2 <= 0
        indicator = 1;
    end
else
    if temp1 <= 0 && temp2 >= 0
        indicator = 1;
    end
end