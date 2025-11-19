% --- MATLAB Script to Construct Rotation Matrices (CORRECTED) ---
% 1. Assuming your active contact normals are stored in obj.normals as COLUMNS
normals_G = obj.normals; % Size is 3x3 (3 components x 3 contacts)
% Pre-allocate a 3x3x3 array to store the three rotation matrices
R_contacts = zeros(3, 3, 3); 
% Iterate through each of the three active contact points (i = 1, 2, 3)
for i = 1:3
    
    % Get the normal vector (points INWARD, defines local Z-axis)
    % CORRECTED: Extract the entire i-th column directly.
    n_i = normals_G(:, i); 
    % n_i is now the 3x1 column vector [n_ix; n_iy; n_iz] for contact i
    
    % --- Step 2: Generate Two Orthogonal Tangential Vectors ---
    
    % Choose an arbitrary reference vector (v) that is NOT collinear with n_i.
    [~, min_idx] = min(abs(n_i));
    v = zeros(3, 1);
    v(min_idx) = 1;
    
    % 2a. Compute the first tangential vector (t_i1) via cross product
    t_i1_unnormalized = cross(n_i, v);
    t_i1 = t_i1_unnormalized / norm(t_i1_unnormalized);
    
    % 2b. Compute the second tangential vector (t_i2) via a second cross product
    t_i2 = cross(n_i, t_i1); 
    
    % --- Step 3: Construct the Rotation Matrix R_i ---
    
    % The rotation matrix R_i transforms the local frame to the Global Frame
    % R_i = [t_i1 | t_i2 | n_i]
    R_i = [t_i1, t_i2, n_i];
    
    % Store the matrix
    R_contacts(:, :, i) = R_i;
end
% R_contacts is now a 3x3x3 array containing the rotation matrices R_1, R_2, R_3.
disp('Rotation matrices R_1, R_2, R_3 are stored in R_contacts.');
disp('R_i = [t_i1 | t_i2 | n_i]');