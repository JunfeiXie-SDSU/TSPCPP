%=========================================================================
% Fast NN-2Opt (Heuristic Algorithm) for TSP-CPP (integrated Traveling Saleman Problem (TSP) and Coverage Path Planning (CPP))
%
% Reference: J. Xie, L. R. G. Carrilo, L. Jin, "Path Planning for UAV to 
%            Cover Multiple  Separated Convex Polygonal Regions", IEEE 
%            Access, Vol. 8, pp. 51770-51785, 2020.
%
% Date: 03/19/2022
%=========================================================================

%=========================================================================

%%%Functionality:
    % Generate the routes for a UAV to cover multiple non-overlapping 
    % convex polygon regions 
    
%%%Input Dataset to be loaded:
    % 'nRegionsCase.mat' stores the map of the regions, where n is the
    % number of regions. The dataset includes 1) 'regions' that stores the
    % coordinates of the vertices of each region, and 2) 'NoofRegions' that
    % specifies the number of regions
%=========================================================================

clear all; clc;
global regions
global NoofRegions
global UAS_range
global UAS_initP

load('HundredRegionsCase.mat');

UAS_range = [1.5,3]; % sensing range of the UAVs;
UAS_initP = [0,0]; %initial position of the UAVs. All UAVs depart from the same location;

% find coverage path for each entrance-exit pair in each region
InterRegionsPoints = FindAllBFPPaths3();

%% initialize the visiting order. 
% find the central point for each region
centralPs = zeros(NoofRegions, 2);
for i = 1:NoofRegions
    rect = regions{i};
    centralPs(i,:) = mean(rect,1);
end
grids = [UAS_initP; centralPs];


[OptimalT,Cost] = tsp_nn(grids); % using nearest neighbor algorithm to initialize the route
[~, c] = find(OptimalT == 1);
OptimalTT = [OptimalT(c:end), OptimalT(1:c-1),1];
% calculate distance table
cities = [UAS_initP; centralPs];
Dtable = zeros(NoofRegions+1, NoofRegions+1);
for i = 1:1+NoofRegions
    dists = vecnorm(kron(cities(i,:), ones(NoofRegions+1,1)) - cities,2,2);
    Dtable(i,:) = dists;
end

%% find the TSP-CPP tour
region_order = OptimalTT(2:end-1) - ones(1,NoofRegions);
[OptRoute, dmin] = FindTourWithRegionOrder2StepM(region_order, InterRegionsPoints,centralPs);
n = NoofRegions + 1;
p = OptimalTT(1:end-1);
cost = Inf;
pmin = p;
while cost - dmin > 1e-2
    cost = dmin;
    i = 0;
    b = p(n);
    % Loop over all edge pairs (ab,cd)
    while i < n-2
        a = b;
        i = i+1;
        b = p(i);
        j = i+1;
        d = p(j);
        while j < n
            c = d;
            j = j+1;
            d = p(j);
            % if (dist(ac) < dist(ab))
            dist1 = Dtable(a,c);
            dist2 = Dtable(a,b);
            if dist1 < dist2
                p_temp = p;
                % alternative path
                p_temp(i:j-1) = p_temp(j-1:-1:i);
                if i == 1
                    p_temp = [p_temp(j-1:end), p_temp(1:j-2)];
                end
                region_order = p_temp(2:end)- ones(1,NoofRegions);
                [Tour_temp, d_temp] = FindTourWithRegionOrder2StepM(region_order, InterRegionsPoints,centralPs);
                % Keep best exchange
                if d_temp < dmin
                    dmin = d_temp;
                    OptRoute = Tour_temp;
                    pmin = p_temp;
                end
            end
        end
    end
    p = pmin;
end


%%%%% plot the trajectory 
figure;
hold on;
for i = 1:NoofRegions
    rectangle = regions{i};
    fill(rectangle(:,1),rectangle(:,2),[0.9,0.9,0.9])
end

distance = 0;
for i = 1:length(OptRoute(:,1))-1
    line(OptRoute(i:i+1,1), OptRoute(i:i+1,2), 'Marker','.', 'LineWidth', 1.5);
    if i >= 2
        distance = distance + norm(OptRoute(i,:)-OptRoute(i+1,:));
    else
        distance = distance + norm(OptRoute(i,:)-OptRoute(i+1,:));
    end
end
plot(OptRoute(1,1), OptRoute(1,2), '>', 'MarkerSize',10, 'MarkerFaceColor', 'r');
hold off;
distance
