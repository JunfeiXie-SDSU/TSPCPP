%%% Given an edge, find the span of a convex polygon 
% reference: coverage path planning for UAVs based on enhanced exact
% cellular decomposition method
% 
% clc; clear all
% 
% numVert = 3;
% center = 10*rand(1,2);
% radius = 5;
% [x,y] = GenerateRandomConvexPolygon(numVert, center, radius);
% plot([x; x(1)],[y; y(1)],'ro-');
% vertex = 2;
function [dis, antipodal, anti_indx] = SpanOfConvexPolygon(x,y,vertex)
numVert = length(x);
% find the span 
if numVert == 3
    current = vertex; 
    next = rem(vertex+1, numVert) + (rem(vertex+1,numVert)==0)*numVert;
    p = rem(vertex+2,numVert)+ (rem(vertex+2,numVert)==0)*numVert;
    edge = [x(current), y(current); x(next), y(next)];
    anti_indx = p;
    antipodal = [x(p), y(p)];
    dis = DisPoint2Edge(antipodal, edge);
else
    current = vertex; 
    next = rem(vertex+1, numVert) + (rem(vertex+1,numVert)==0)*numVert;
    p = rem(vertex+1,numVert)+ (rem(vertex+1,numVert)==0)*numVert;
    p_next = rem(vertex+2,numVert)+ (rem(vertex+2,numVert)==0)*numVert;
    edge = [x(current), y(current); x(next), y(next)];

    dij = (y(next)-y(current))*(x(p_next)-x(p)) - (x(next)-x(current))*(y(p_next)-y(p));
    d_temp = sign(dij);
    direction = d_temp;
    % find the antipodal vertex of edge (vi, v_i+1)
    while direction == d_temp
        p = rem(p+1,numVert)+ (rem(p+1,numVert)==0)*numVert;
        p_next = rem(p+1,numVert)+ (rem(p+1,numVert)==0)*numVert;
        dij = (y(next)-y(current))*(x(p_next)-x(p)) - (x(next)-x(current))*(y(p_next)-y(p));
        direction = sign(dij);
    end
    anti_indx = p;
    antipodal = [x(p), y(p)]; 
    % calculate the distance between the antipodal and the edge
    dis = DisPoint2Edge(antipodal, edge);
end