function [pts3D] = algebraicTriangulation(pts2D_1, pts2D_2, ProjMat_1, ProjMat_2)
%ALGEBRAICTRIANGULATION(pts2D_1, pts2D_2, ProjMat_1, ProjMat_2) Gives 3D
%points using the information of 2D points from two images and the
%corresponding projection matrices.
%   Inputs:
%   pts2D_1: Corresponding points from image 1
%   pts2D_2: Corresponding points from image 2
%   ProjMat_1: Projection matrix for image 1
%   ProjMat_2: Projection matrix for image 2
%   Outputs:
%   pts3D: Estimated 3D point

    pts3D = zeros(4,size(pts2D_1,2));
    for i = 1:size(pts2D_1,2)
        A = [pts2D_1(1,i)*ProjMat_1(3,:) - ProjMat_1(1,:);...
             pts2D_1(2,i)*ProjMat_1(3,:) - ProjMat_1(2,:);...
             pts2D_2(1,i)*ProjMat_2(3,:) - ProjMat_2(1,:);...
             pts2D_2(2,i)*ProjMat_2(3,:) - ProjMat_2(2,:)];
        [~,~,v] = svd(A);
        pt = v(:,end);
        pts3D(:,i) = pt/pt(4,1);
    end    
    
end

