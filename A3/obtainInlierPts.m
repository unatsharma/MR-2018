function [inlierPts_1,inlierPts_2] = obtainInlierPts(F,pts1,pts2,threshold)
%OBTAININLIERPTS(F,PTS1,PTS2) Summary of this function goes here
%   Detailed explanation goes here

    indices = find(abs(sum(pts2.*(F*pts1')',2)) < threshold);
    inlierPts_1 = pts1(indices,:);
    inlierPts_2 = pts2(indices,:);
    
end

