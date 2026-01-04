% Export discrete-time matrices from prep_final.m for Python comparison
% 
% This script runs prep_final.m and exports the discrete matrices
% to a .mat file for numerical comparison with Python implementation.

% Run prep_final.m to get discrete matrices
run('prep_final.m');

% Extract Dd matrix (should be zeros since D=zeros(size(C,1), size(B,2)))
% Dd is needed for complete comparison even though it's all zeros
Dd = sys_discrete.D;

% Export discrete matrices to .mat file
save('matlab/discrete_matrices.mat', 'Ad', 'Bd', 'Cd', 'Dd', 'Ts', 'x0', 'N');

fprintf('Exported discrete matrices to matlab/discrete_matrices.mat\n');
fprintf('Ad shape: %d x %d\n', size(Ad));
fprintf('Bd shape: %d x %d\n', size(Bd));
fprintf('Cd shape: %d x %d\n', size(Cd));
fprintf('Dd shape: %d x %d\n', size(Dd));

