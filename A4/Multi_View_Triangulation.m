close all;
clc;

% Load the image points and projection matrices
load('cube_imgs.mat');
load('projMatrices.mat');

% X*P = x
[p,r,c] = size(image_pts(:,:,:));
best_X = zeros(4,c);
for i = 1:7
    for j = i+1:8        
        ip_1 = reshape(image_pts(i,:,:),[r,c]);
        ip_2 = reshape(image_pts(j,:,:),[r,c]);
        pm_1 = projMatrices{i};
        pm_2 = projMatrices{j};
        
        X = zeros(4,c);
        for k = 1:c
            X(:,k) = triangulation(ip_1(:,k), ip_2(:,k), pm_1, pm_2);
            best_X = update_bestX(X(:,k), best_X, k, projMatrices, image_pts);
        end
    end
end

% Plot the 3D points
figure;
scatter3(best_X(1,:),best_X(2,:),best_X(3,:),'x');
title('Reconstruction cube after using all the views for triangulation');
legend('Cube points');
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');

% Find 3D point
function X = triangulation(ip_1, ip_2, pm_1, pm_2)
    A = [ip_1(1)*pm_1(3,:) - pm_1(1,:);...
         ip_1(2)*pm_1(3,:) - pm_1(2,:);...
         ip_2(1)*pm_2(3,:) - pm_2(1,:);...
         ip_2(2)*pm_2(3,:) - pm_2(2,:)];
    [~,~,v] = svd(A);
    pt = v(:,end);
    X = pt/pt(4,1);
end

% Update best 3D point
function b_X = update_bestX(X, b_X, k, projMatrices, image_pts)
    flag = true;
    for i = 1:8
        x = projMatrices{i} * X;
        x = x/x(3);
        x = x(1:2);
        y = image_pts(i,:,k)';
        if (~isequal(round(x,4),round(y,4)))
            flag = false;
        end
    end
    
    if (flag)
        b_X(:,k) = X;
    end    
end