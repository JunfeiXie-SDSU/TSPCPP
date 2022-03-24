function [optR, optC] = findEntraExit2(regionIndx, ex, nextRegion, InterRegionsPoints,C_Sji,seq_sji)
global UAS_initP
if nextRegion == 0
        seq_temp = InterRegionsPoints{regionIndx}{ex};
        optR = seq_temp(1:end-1,:);
        if size(optR,1) > 1
        optC = seq_temp(end,1) + norm(seq_temp(end-1,:) - UAS_initP);
        else
            optC = seq_temp(end,1) + norm(seq_temp(end-1,:) - UAS_initP);
        end
else
        NumNexthop = length(C_Sji);
        seq_temp = InterRegionsPoints{regionIndx}{ex};
        OptRoute = seq_temp(1:end-1,:);
        Cost = seq_temp(end,1);
        totalcost = zeros(NumNexthop,1);
        for j = 1:NumNexthop
            totalcost(j) = Cost + norm(OptRoute(end,:) - seq_sji{j}(1,:)) + C_Sji(j);
        end
        [r, ~] = find(totalcost == min(totalcost));
        r = r(1);
        optR = [OptRoute; seq_sji{r}];
        optC = totalcost(r);
end