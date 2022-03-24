function InterRegionPoints = FindAllBFPPaths3(NoofRegions, regions)
global UAS_range % l x w

InterRegionPoints =  cell(NoofRegions, 1);

%InterRegionPoints{i}  -> i-th region
%InterRegionPoints{i}{4j-3} -> coverage path that starts at vertex j and then j+1
%InterRegionPoints{i}{4j-2} -> reverse of path InterRegionPoints{i}{4j-3}
%InterRegionPoints{i}{4j-1} -> coverage path that starts at vertex j+1 and then j
%InterRegionPoints{i}{4j} -> reverse of path InterRegionPoints{i}{4j-1}

% The last row of InterRegionPoints{i}{j} gives the cost of the path

for i = 1:NoofRegions
    region = regions{i};
    numVert = size(region, 1);
    InterRegionPoints{i} = cell(numVert*4, 1);  
    for j = 1:numVert
        if j < numVert
            jn = j+1;
        else
            jn = 1;               
        end
        [path, cost] = getPath5(region(:,1), region(:,2), j, jn, UAS_range);
        InterRegionPoints{i}{4*j-3} = [path{1}; cost(1), 0];
        InterRegionPoints{i}{4*j-2} = [path{1}(end:-1:1,:); cost(1), 0];
        InterRegionPoints{i}{4*j-1} = [path{2}; cost(2), 0];
        InterRegionPoints{i}{4*j} = [path{2}(end:-1:1,:); cost(2), 0];
    end
end