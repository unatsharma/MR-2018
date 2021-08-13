function [match_Pts1,match_Pts2] = findCorrespondingPts(image1,image2)
%FINDCORRESPONDINGPTS(IMAGE1,IMAGE2) Finds corresponding points from the
%two images
%   Inputs:
%   image1: Image 1
%   image2: Image 2
%   Outputs:
%   match_Pts1: Corresponding points from image 1
%   match_Pts2: Corresponding points from image 2

    pts1 = detectSURFFeatures(image1,'MetricThreshold',10);
    pts2 = detectSURFFeatures(image2,'MetricThreshold',10);

    [f1,vpts1] = extractFeatures(image1,pts1);
    [f2,vpts2] = extractFeatures(image2,pts2);

    indexPairs = matchFeatures(f1,f2);
    match_Pts1 = vpts1(indexPairs(:,1));
    match_Pts2 = vpts2(indexPairs(:,2));

end

