clear all;
close all;
clc;

%Creating a surface i.e., infinite paraboloid.
syms u v
x = u;
y = v;
z = u^2+v^2;

subplot(1,2,1);
fsurf(x,y,z);
title('Original image');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

%Rotation matrix rotating about the x-axis.
syms angle
Rx = [1 0 0;0 cos(angle) -sin(angle);0 sin(angle) cos(angle)];

planeRotatedAboutX = Rx*[x;y;z];
xRotation = subs(planeRotatedAboutX,angle,pi);

%Image after application of rotation.
subplot(1,2,2);
fsurf(xRotation(1),xRotation(2),xRotation(3));
title('Image after 180 degree rotation about x-axis');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');