% clear all;
% close all;
clc;

subplot(1,2,1);
% fsurf(x,y,z1);
x = (-50:50);
y = zeros(1,101);
z1 = x+10;
z2 = x-10;
plot3(x,y,z1,x,y,z2);
title('Pair of paralle lines')
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

camMatrix = [500 0 320;0 500 240;0 0 1]*[0.985 -0.151 -0.087 10;0 -0.5 0.866 -30;-0.174 -0.853 -0.492 40];
projection1 = camMatrix*[x;y;z1;ones(1,101)];
projection2 = camMatrix*[x;y;z2;ones(1,101)];

subplot(1,2,2);
x1 = bsxfun(@rdivide,projection1(1,:),projection1(3,:));
y1 = bsxfun(@rdivide,projection1(2,:),projection1(3,:));
x2 = bsxfun(@rdivide,projection2(1,:),projection2(3,:));
y2 = bsxfun(@rdivide,projection2(2,:),projection2(3,:));

plot(x1,y1,x2,y2);
xlim([0,1200]);
title('Projection on 2D')
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');