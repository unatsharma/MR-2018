%Read the images.
image1 = imread('img3.png');
image2 = imread('img4.png');
image3 = imread('img5.png'); 

%Detect the corners of the checkerboard images.
imagePoints1 = detectCheckerboardPoints(image1);
imagePoints2 = detectCheckerboardPoints(image2);
imagePoints3 = detectCheckerboardPoints(image3);

%Generate the actual corners of the checkerboard.
sizeOfBoard = [6,8];
sideLength = 2.4;
actualPoints = generateActualCheckerboardPoints(sizeOfBoard,sideLength);

%Calculating homography matrix for each image.
actualPoints_1 = [actualPoints(2,1),actualPoints(2,2);actualPoints(6,1),...
    actualPoints(6,2);actualPoints(48,1),actualPoints(48,2);actualPoints(43,1),actualPoints(43,2)];

imagePoints_1 = [imagePoints1(2,1),imagePoints1(2,2);imagePoints1(6,1),...
    imagePoints1(6,2);imagePoints1(48,1),imagePoints1(48,2);imagePoints1(43,1),imagePoints1(43,2)];

imagePoints_2 = [imagePoints2(2,1),imagePoints2(2,2);imagePoints2(6,1),...
    imagePoints2(6,2);imagePoints2(48,1),imagePoints2(48,2);imagePoints2(43,1),imagePoints2(43,2)];

imagePoints_3 = [imagePoints3(2,1),imagePoints3(2,2);imagePoints3(6,1),...
    imagePoints3(6,2);imagePoints3(48,1),imagePoints3(48,2);imagePoints3(43,1),imagePoints3(43,2)];

H_matrix_1 = generateHomographyMatrix(actualPoints_1,imagePoints_1);
H_matrix_2 = generateHomographyMatrix(actualPoints_1,imagePoints_2);
H_matrix_3 = generateHomographyMatrix(actualPoints_1,imagePoints_3);

%Estimating K matrix from the above obtained homography matrices.
K = generateCalibrationMatrix(H_matrix_1,H_matrix_2,H_matrix_3);
disp('K');
disp(K);

%Estimating the rotation matrix and translation for each camera position.
r_1 = generateRotationMatrix(K,H_matrix_1);
r_2 = generateRotationMatrix(K,H_matrix_2);
r_3 = generateRotationMatrix(K,H_matrix_3);
t_1 = K\H_matrix_1(:,3);
t_2 = K\H_matrix_2(:,3);
t_3 = K\H_matrix_3(:,3);
T_1 = [r_1, t_1;0,0,0,1];
T_2 = [r_2, t_2;0,0,0,1];
T_3 = [r_3, t_3;0,0,0,1];

%Reprojecting the image points.
reprojImgPts_1 = reprojectImgPoints(actualPoints,H_matrix_1);
reprojImgPts_2 = reprojectImgPoints(actualPoints,H_matrix_2);
reprojImgPts_3 = reprojectImgPoints(actualPoints,H_matrix_3);

%Calculating the error.
meanSquareError_1 = reprojectionError(actualPoints,imagePoints1,H_matrix_1);
meanSquareError_2 = reprojectionError(actualPoints,imagePoints2,H_matrix_2);
meanSquareError_3 = reprojectionError(actualPoints,imagePoints3,H_matrix_3);

%Show the images with corners.
subplot(1,3,1);
imshow(image1);
hold on;
plot(imagePoints1(:,1),imagePoints1(:,2),'rx');
plot(reprojImgPts_1(:,1),reprojImgPts_1(:,2),'b.');
hold off;
title('Image 3');
subplot(1,3,2);
imshow(image2);
hold on;
plot(imagePoints2(:,1),imagePoints2(:,2),'rx');
plot(reprojImgPts_2(:,1),reprojImgPts_2(:,2),'b.');
hold off;
title('Image 4');
subplot(1,3,3);
imshow(image3);
hold on;
plot(imagePoints3(:,1),imagePoints3(:,2),'rx');
plot(reprojImgPts_3(:,1),reprojImgPts_3(:,2),'b.');
hold off;
title('Image 5');
figure;
bar([meanSquareError_1,meanSquareError_2,meanSquareError_3]);
xlabel('Image number');
ylabel('MSE in reprojection points obtained by using the camera intrinsics');

%Generate actual checkerboard points in the world coordinates.
function [actualPoints] = generateActualCheckerboardPoints(sizeOfBoard,sideLength)
    totalCorners = sizeOfBoard(1)*sizeOfBoard(2);
    actualPoints = zeros(totalCorners,2);
    
    index = 1;
    for i = 1:sizeOfBoard(2)
        for j = 1:sizeOfBoard(1)
            x_coord = i*sideLength;
            y_coord = j*sideLength;
            actualPoints(index,:) = [x_coord;y_coord];
            index = index + 1;
        end
    end
end

%Generate the homography matrix for the given world coordinate points and
%the image points.
function [homography] = generateHomographyMatrix(actualPoints,imagePoints)
    A = zeros(8);    
    B = zeros(8,1);
    
    for i = 1:4
        actual_x = actualPoints(i,1);
        actual_y = actualPoints(i,2);
        image_x = imagePoints(i,1);
        image_y = imagePoints(i,2);
        
        j = 2*i;
        A(j-1,:) = [actual_x,actual_y,1,0,0,0,(-1*actual_x*image_x),(-1*actual_y*image_x)];
        A(j,:) = [0,0,0,actual_x,actual_y,1,(-1*actual_x*image_y),(-1*actual_y*image_y)];
        B(j-1) = image_x;
        B(j) = image_y;
    end
    
    X = linsolve(A,B);
    
    homography = [X(1,1),X(2,1),X(3,1);X(4,1),X(5,1),X(6,1);X(7,1),X(8,1),1];    
end

%Estimating K matrix from H matrix.
function K_matrix = generateCalibrationMatrix(homography_1,homography_2,homography_3)
    V_1 = calculateVMatrix(homography_1);
    V_2 = calculateVMatrix(homography_2);
    V_3 = calculateVMatrix(homography_3);
    
    V = [V_1;V_2;V_3];
    
    [evec,eval] = eig(transpose(V)*V);
    [d,ind] = sort(diag(eval));
    b = evec(:,ind(1,1));
    
    B = zeros(3);
    B(1,1) = b(1,1);
    B(1,2) = b(2,1);
    B(1,3) = b(3,1);
    B(2,1) = B(1,2);
    B(2,2) = b(4,1);
    B(2,3) = b(5,1);
    B(3,1) = B(1,3);
    B(3,2) = B(2,3);
    B(3,3) = b(6,1);
    
    K_matrix = inv(chol(B));
    K_matrix = K_matrix/K_matrix(3,3);   
end

%Calculates V matrix corresponding to a homography matrix.
function v_matrix = calculateVMatrix(homography_matrix)
    v_11 = vElement(homography_matrix,1,1);
    v_12 = vElement(homography_matrix,1,2);
    v_22 = vElement(homography_matrix,2,2);
    v_matrix = [v_12; v_11-v_22];
end

%Create particular element of the v-matrix.
function v = vElement(homography_matrix,index_1,index_2)
    v = [homography_matrix(1,index_1)*homography_matrix(1,index_2),...
        homography_matrix(1,index_1)*homography_matrix(2,index_2)+homography_matrix(2,index_1)*homography_matrix(1,index_2),...
        homography_matrix(3,index_1)*homography_matrix(1,index_2)+homography_matrix(1,index_1)*homography_matrix(3,index_2),...
        homography_matrix(2,index_1)*homography_matrix(2,index_2),...
        homography_matrix(3,index_1)*homography_matrix(2,index_2)+homography_matrix(2,index_1)*homography_matrix(3,index_2),...
        homography_matrix(3,index_1)*homography_matrix(3,index_2)];
end

%Calculate R matrix for the given homography and calibration matrix.
function R = generateRotationMatrix(cal_matrix,homo_matrix)
    r_1 = cal_matrix\homo_matrix(:,1);
    r_1 = r_1/norm(r_1);
    r_2 = cal_matrix\homo_matrix(:,2);
    r_2 = r_2/norm(r_2);
    r_3 = cross(r_1,r_2);
    R = [r_1, r_2, r_3];
end

%Reporject the points in the image using H matrix.
function reproj_pts = reprojectImgPoints(actualPts,H)
    [rows,cols] = size(actualPts);
    reproj_pts = zeros(rows,cols);
    
    for i = 1:rows
        pt = H * [transpose(actualPts(i,:));1];
        pt = pt/pt(3,1);
        reproj_pts(i,1) = pt(1,1);
        reproj_pts(i,2) = pt(2,1);
    end 
end

%Generate reprojection error corresponding to an image and the camera
%intrinsics.
function error = reprojectionError(actual,image,H)
    [rows,cols] = size(actual);
    error = 0;
    
    for i = 1:rows        
        pt = H*[transpose(actual(i,:));1];
        pt = pt/pt(3,1);
        
        error = error + (image(i,1)-pt(1,1))^2 + (image(i,2)-pt(2,1))^2;        
    end
    
    error = error/rows;
end