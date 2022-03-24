function [optRoute,minDist] = tsp_nn(xy)
popSize = Inf;    
nPoints = size(xy,1);
a = meshgrid(1:nPoints);
dmat = reshape(sqrt(sum((xy(a,:)-xy(a',:)).^2,2)),nPoints,nPoints);

[N,~] = size(xy);
n = N;

% Sanity Checks
popSize     = max(1,min(n,round(real(popSize(1)))));

% Initialize the Population
pop = zeros(popSize,n);

% Run the NN
distHistory = zeros(1,popSize);
for p = 1:popSize
    d = 0;
    thisRte = zeros(1,n);
    visited = zeros(1,n);
    I = p;
    visited(I) = 1;
    thisRte(1) = I;
    for k = 2:n
        dists = dmat(I,:);
        dists(logical(visited)) = NaN;
        dMin = min(dists(~visited));
        J = find(dists == dMin,1);
        visited(J) = 1;
        thisRte(k) = J;
        d = d + dmat(I,J);
        I = J;
    end
    d = d + dmat(I,p);
    pop(p,:) = thisRte;
    distHistory(p) = d;

end

% Find the Minimum Distance Route
[minDist,index] = min(distHistory);
optRoute = pop(index,:);   
end