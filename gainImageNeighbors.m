function imageNeighbors = gainImageNeighbors(input, numMatches)

    %%***********************************************************************%
    %*                   Automatic panorama stitching                       *%
    %*                        Bundle adjustment                             *%
    %*                                                                      *%
    %* Code author: Preetham Manjunatha                                     *%
    %* Github link: https://github.com/preethamam                           *%
    %* Date: 01/27/2022                                                     *%
    %************************************************************************%
    
    % Find connected components of image matches
    numMatchesG = graph(numMatches,'upper');
    [concomps, ccBinSizes] = conncomp(numMatchesG);
    
    % Find images neighbors    
    nearestFeaturesNum = input.nearestFeaturesNum;
    unqCCs = unique(concomps,'stable');
    imageNeighbors = cell(numel(unqCCs),1);
    for j = 1:length(unqCCs)
        ccIdx = find(concomps == unqCCs(j));
        numMatchesCCs = numMatches(ccIdx, ccIdx);
        numMatchesGCCs = graph(numMatchesCCs,'upper');
        if size(numMatchesGCCs.Edges,1) == 1
            imageNeighbors{j} = {numMatchesGCCs.Edges.EndNodes};
            continue
        end
        parfor i = 1:size(numMatchesCCs,1)
            nn = neighbors(numMatchesGCCs,i)';
            nn_dist = distances(numMatchesGCCs,i, nn);
            imageNeighborsTemp{i} = nn(nn_dist>=nearestFeaturesNum);
        end
            imageNeighbors{j} = imageNeighborsTemp;
    end
end