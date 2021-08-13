%Calculate transformation matrix, rotation matrix and translation matrix
clear all;
clc;

B = [3 4 5 1;-3 4 -5 1;1 3 5 1;3 2 -4 1];
Xa1 = [9.654; -1.4118; 7.8994; 1.9410];
Xa2 = [2.1513; 5.7623; 0.7050; 5.8320];
Xa3 = [-0.9041; -0.1970; -0.1022; -5.3108];
   
R1 = linsolve(B,Xa1);
R2 = linsolve(B,Xa2);
R3 = linsolve(B,Xa3);
R4 = [0 0 0 1];

TransformationMatrix = [transpose(R1);transpose(R2);transpose(R3);R4]

RotationMatrix = TransformationMatrix(1:3,1:3);
translationMatrix = TransformationMatrix(1:3,4);

eulerZYX = rotm2eul(RotationMatrix);
