function [F, inliers_1, inliers_2] = estimateFundamentalMatrixRANSAC(corr_pts1, corr_pts2, T1, T2)
%estimateFundamentalMatrixRANSAC Estimates fundamental matrix using RANSAC
%   Inputs:
%   corr_pts1:  Corresponding points of image 1
%   corr_pts2:  Corresponding points of image 2
%   T1:         Transformation for points of image 1
%   T2:         Transformation for points of image 2

    % Normalizing the homogeneous coordinates with respect to third
    % coordinate
    norm_pts1 = corr_pts1 * T1';
    norm_pts2 = corr_pts2 * T2';
    
    % RANSAC
    itrs = 300;
    ptsForF = 8;
    errThreshold = 0.01;
    f = ones(itrs,3,3);
    inliers = ones(itrs,1);

    for itr = 1:itrs
        % Select points randomly
        pts = randi(size(corr_pts1,1),[ptsForF,1]);
        
        % Finding F for x1 <-> x2
        f_m = calculateF(norm_pts1(pts,:),norm_pts2(pts,:));
        f(itr,:,:) = f_m;  
        inliers(itr) = size(find(abs(sum(norm_pts2.*(f_m*norm_pts1')',2)) < errThreshold),1);
    end
    
    [~,max_index] = max(inliers);
    Fn = reshape(f(max_index,:,:),3,3);
    
    [in_1,in_2] = obtainInlierPts(Fn,norm_pts1,norm_pts2,errThreshold);
    inliers_1 = in_1 * inv(T1');
    inliers_2 = in_2 * inv(T2');
    F = T2'*calculateF(inliers_1*T1',inliers_1*T2')*T1;
end

function [F_matrix] = calculateF(corrPts1,corrPts2)
% calculateF Calculates F matrix for the given corresponding points from
% two images
    % AF = 0   
    % x2^T*F*x1 = 0
    noOfPts = size(corrPts1,1);
    A_matrix = [corrPts2(:,1).*corrPts1(:,1) corrPts2(:,1).*corrPts1(:,2) corrPts2(:,1) ...
                corrPts2(:,2).*corrPts1(:,1) corrPts2(:,2).*corrPts1(:,2) corrPts2(:,2) ...
                corrPts1(:,1) corrPts1(:,2) ones(noOfPts,1)];
            
    [~,~,V] = svd(A_matrix);    
    F_matrix = reshape(V(:,end),[3,3]);
    
    [U,S,V] = svd(F_matrix); 
    S(3,3) = 0;
    F_matrix = U*S*V';
end