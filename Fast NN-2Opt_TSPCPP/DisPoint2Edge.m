function distance = DisPoint2Edge(point, edge)
x1 = edge(1,1);
x2 = edge(2,1);
y1 = edge(1,2);
y2 = edge(2,2);
temp = abs((y2-y1)*point(1) - (x2-x1)*point(2) + x2*y1 - x1*y2);
distance = temp/sqrt((y2-y1)^2 + (x2-x1)^2);
end