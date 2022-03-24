%=========================================================================
% Dynamic Algorithm for TSP-CPP (integrated Traveling Saleman Problem (TSP) and Coverage Path Planning (CPP))
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
global UAS_range
global UAS_initP
tic;

load 'ThreeRegionsCase.mat'
UAS_range = [1.5,3]; % sensing range of the UAVs;
UAS_initP = [10,10]; %initial position of the UAVs. All UAVs depart from the same location;

% find coverage path for each entrance-exit pair in each region
InterRegionsPoints = FindAllBFPPaths3(NoofRegions, regions);

Set = cell(NoofRegions, 1);
Cost = cell(NoofRegions, 1);
Seq = cell(NoofRegions, 1);

for i = 1:NoofRegions
    Comb = nchoosek(1:NoofRegions, i);
    NoofSet = length(Comb(:,1));
    Set{i} = cell(NoofSet, 1);
    Cost{i} = cell(NoofSet, 1);
    Seq{i} = cell(NoofSet, 1);
    % 1st index {i}: access number of regions considered
    % 2nd index {k}: each possible combination
    % 3rd index: each possible region to end 
    % 4th index: extrance of each region
    for k = 1:NoofSet
        comb = nchoosek(Comb(k,:), 1);
        noofset = length(comb(:,1));
        Set{i}{k} = cell(noofset, 1);
        Cost{i}{k} = cell(noofset, 1);
        Seq{i}{k} = cell(noofset, 1);
        for t = 1:noofset
            endRegion = comb(t);
            numEEPairs = size(regions{endRegion},1)*4;   
            Set{i}{k}{t} = [ones( numEEPairs, 1)*[Comb(k,:), comb(t)], (1: numEEPairs)'];
            Cost{i}{k}{t} = inf * ones(1,  numEEPairs);
            Seq{i}{k}{t} = cell( numEEPairs, 1);
        end
    end
end

for t = 1:NoofRegions 
    for i = 1:length(Set{t})
        for k = 1:length(Set{t}{i})
            set_temp = Set{t}{i}{k}(1,1:end-1);
            endRegion = set_temp(end);
            currRegion = regions{endRegion};
            numEEPairs = size(currRegion, 1)*4;  
            regionsToConsider = set_temp(1:end-1);
            % find regions that have been visited
            [~,index2] = find(regionsToConsider ~= endRegion);
            regionsVisited = regionsToConsider(index2);
            for ex = 1: numEEPairs
                if isempty(regionsVisited)
                    nextState = UAS_initP;
                    % calculate the distance and sequence for each region
                    [optR, optC] = findEntraExit2(endRegion, ex, 0, InterRegionsPoints);
                    Cost{t}{i}{k}(ex) = optC;
                    Seq{t}{i}{k}{ex} = [optR; UAS_initP];
                else 
                    % find the minimum cost 
                    seq_candi = cell(1,length(regionsVisited));
                    cost_candi = zeros(1, length(regionsVisited));
                    for m = 1:length(regionsVisited)
                        region_i = regionsVisited(m); % i in the above equation
                        nextRegion = regions{region_i};
                        temp = [regionsVisited, region_i];
                        f = false;
                        ii = 1;
                        kk = 1;
                        % find record in Set that match temp
                        for ii = 1:length(Set{t-1}) 
                            for kk = 1:length(Set{t-1}{ii})
                                f = isequal(sort(temp),sort(Set{t-1}{ii}{kk}(1,1:end-1)));
                                if f == true
                                    index3 = ii;
                                    break;
                                end
                            end
                            if f == true
                                break;
                            end
                        end
                        C_Sji = Cost{t-1}{index3}{kk};
                        seq_sji = Seq{t-1}{index3}{kk};
                        [optR, optC] = findEntraExit2(endRegion, ex, nextRegion, InterRegionsPoints,C_Sji,seq_sji);
                        cost_candi(m) = optC;
                        seq_candi{m} = optR;
                    end
                    %%% find C(S,j) and corresponding sequence
                    [~, index1] = find(cost_candi == min(cost_candi));
                    index1 = index1(1);
                    %%%% update Cost, and Seq
                    Cost{t}{i}{k}(ex) = cost_candi(index1);
                    Seq{t}{i}{k}{ex} = seq_candi{index1};
                end
            end
        end
    end
end
% add distance from the last region back to UAS 
for i = 1:NoofRegions
    numEEPairs = size(regions{i}, 1)*4; 
    for j = 1:numEEPairs
        Seq{end}{end}{i}{j} = [UAS_initP; Seq{end}{end}{i}{j}];
        v1 = Seq{end}{end}{i}{j}(2,:) - Seq{end}{end}{i}{j}(1,:);
        v2 = Seq{end}{end}{i}{j}(3,:) - Seq{end}{end}{i}{j}(2,:);
        theta = acos(dot(v1, v2)/norm(v1)/norm(v2));
        Cost{end}{end}{i}(j) = Cost{end}{end}{i}(j) + norm(Seq{end}{end}{i}{j}(2,:) ...
            - Seq{end}{end}{i}{j}(1,:));
    end
end

costE = zeros(1, NoofRegions);
seqE = cell(NoofRegions, 1);
%%% find the optimal route
for i = 1:NoofRegions
    [~, index6] = find(Cost{end}{end}{i} == min(Cost{end}{end}{i}));
    index6 = index6(1);
    costE(i) = Cost{end}{end}{i}(index6);
    seqE{i} = Seq{end}{end}{i}{index6};
end

[~, index4] = find(costE == min(costE));
OptRoute = seqE{index4};
MinCost = costE(index4);

% %%%%% plot the trajectory 
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



