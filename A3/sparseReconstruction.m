clear all;
close all;
clc;

%%%%%%%%%%%%%%%%% Step 1 %%%%%%%%%%%%%%%%%
image1 = rgb2gray(imread('img1.png'));
image2 = rgb2gray(imread('img2.png'));

% Obtain corresponding points from the two images
[match_Pts1,match_Pts2] = findCorrespondingPts(image1,image2);

%%%%%%%%%%%%%%%%% Step 2 %%%%%%%%%%%%%%%%%
pts1 = match_Pts1.Location;
pts2 = match_Pts2.Location;
[norm_Pts1,T1] = normalize2DPoints(pts1);
[norm_Pts2,T2] = normalize2DPoints(pts2);

%%%%%%%%%%%%%%%%% Step 3 %%%%%%%%%%%%%%%%%
new_Pts1 = [pts1,ones(size(pts1,1),1)];
new_Pts2 = [pts2,ones(size(pts2,1),1)];
[F_matrix, inliers_1, inliers_2] = estimateFundamentalMatrixRANSAC(new_Pts1, new_Pts2, T1, T2);

%%%%%%%%%%%%%%%%% Step 4 %%%%%%%%%%%%%%%%%
K_matrix = [558.7087, 0.0000, 310.3210;...
            0.0000, 558.2827, 240.2395;...
            0.0000, 0.0000, 1.0000];
E_matrix = calcEssentialMatrix(F_matrix',K_matrix);
[R,t] = decomposeEssentialMatrix(E_matrix, inliers_1', inliers_2', K_matrix);


%%%%%%%%%%%%%%%%% Step 5 %%%%%%%%%%%%%%%%%
ProjMat_1 = K_matrix*[eye(3,3) [0,0,0]'];
ProjMat_2 = K_matrix*[R -R*t];

pts3D = algebraicTriangulation(inliers_1',inliers_2',ProjMat_1,ProjMat_2);


%%%%%%%%%%%%%%%%% Step 6 %%%%%%%%%%%%%%%%%
disp('F matrix:');
disp(F_matrix');
disp('R:');
disp(R);
disp('t:');
disp(t);

% 'ro' matchPts1
% 'g+' matchPts2
% Plotting the points in the images
showMatchedFeatures(image1,image2,inliers_1(:,1:2),inliers_2(:,1:2),'montage');

% Triangulated points.
figure;
scatter3(pts3D(1,:),pts3D(2,:),pts3D(3,:),'.');
title('Triangulated points using projection matrices');
colormap;
hold on;
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
plotCameraFrustum([eye(3,3) [0,0,0]'; [0,0,0,1]],'m',0.05);
plotCameraFrustum([R t; [0,0,0,1]],'g',0.05);
hold off;