%%%% Given the visiting order of the regions, find the best intra-regional
%%%% path and entrance/exit locations using modified Two-Step
function [OptRoute, MinCost] = FindTourWithRegionOrder2StepM(region_order, InterRegionsPoints, centralPs)
global regions
global NoofRegions
global UAS_initP

OptimalTT = [1,region_order + ones(1,NoofRegions), 1];
%% find the entrance and exit points at each region
% then find the inter-regional path
% initialize
OptRoute = UAS_initP;
distance = 0;
% find optimal tour
for i = 2:length(OptimalTT)-1
    currRegion = OptimalTT(i);
    preRegion = OptimalTT(i-1);
    if preRegion == 1
        prePoint = UAS_initP;
    else
        prePoint = OptRoute(end,:);
    end
    nextRegion = OptimalTT(i+1);
    if nextRegion == 1
        nextPoint = UAS_initP;
    else
        nextPoint = centralPs(nextRegion-1,:);
    end
    numEEPairs = size(regions{currRegion-1}, 1)*4;
    currCost = zeros(numEEPairs, 1);
    for j = 1:numEEPairs
        tour = InterRegionsPoints{currRegion-1}{j};
        currCost(j) = norm(prePoint - tour(1,:)) + tour(end,1) + norm(tour(end-1,:) - nextPoint);
    end
    [nr,~] = find(currCost == min(currCost));
    OptRoute = [OptRoute; InterRegionsPoints{currRegion-1}{nr(1)}(1:end-1,:)];
    distance = distance + InterRegionsPoints{currRegion-1}{nr(1)}(end,1) + norm(prePoint - InterRegionsPoints{currRegion-1}{nr(1)}(1,:));
end
OptRoute = [OptRoute; UAS_initP];
distance = distance + norm(OptRoute(end-1,:)-OptRoute(end,:));
MinCost = distance;
end