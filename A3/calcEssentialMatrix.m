function [E_matrix] = calcEssentialMatrix(F_matrix,K_matrix)
%CALCESSENTIALMATRIX(F_MATRIX,K_MATRIX) Estimates Essential matrix from
%given Fundamental matrix and calibration matrix
%   Inputs:
%   F_matrix: Fundamental matrix
%   K_matrix: Calibration matrix
%   Output:
%   E_matrix: Essential matrix


    E_matrix = K_matrix'*F_matrix*K_matrix;
    
    % Making Essential matrix of rank 2 and making both its singular values
    % equal
    [U,D,V] = svd(E_matrix);
    new_D = diag([(D(1,1)+D(2,2))/2, (D(1,1)+D(2,2))/2, 0]);    
    E_matrix = U * new_D * V';
end

