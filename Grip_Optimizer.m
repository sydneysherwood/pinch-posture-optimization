%yalmip('clear');
%clear;
%clc;



disp('Setting up the optimization problem...');
load('allegro_data.mat')

%load('fake_allegro_data.mat');
G = Grasp_Matrix;
J = Hand_Jacobian;

%UNITS -- G AND J ARE IN MM OR N/MM WHERE A FORCE IS INVOLVED
disp('Setting up the optimization problem...');


% External wrench [Fx, Fy, Fz, Mz, My, Mz]' applied *on* the object

%DEFINE W_EXT IN N/MM
%0.00098 = 100 g weight operated on by gravity
%0.
w_ext = [-5; 0; -10; 0; 0; 0];

% Joint torque limits [ABSOLUTE VALUE] -- from Allergo website
%tau_max = [0.7 0.7 0.7 0.7]';

%CONVERT TO N/MM
tau_max = [0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7]'.*1000;
% Friction coefficient (mu) -- PURELY ARBITRARY FOR NOW
mu = 1.3;

%DECISION VARIABLES

%sdpvar --> symbolic decision variables
%inputs are length, height --> e.g. 1,1 is a scalar var

% t: The (6x1) vector of contact forces we are solving for (3 contact
% forces for each contact point -- 2 tangential and 1 normal force
% s: The scalar "maximal cost index" we want to minimize
s = sdpvar(1, 1); 
t = sdpvar(9, 1);
G
%DEFINE CONSTRAINTS
Constraints = [];

%GRASP EQUILIBRIUM: force equilibrium constraint: grasp matrix * contact 
% forces must equal applied forces
Constraints = [Constraints, G * t == -w_ext]; 

%JOINT TORQUE LIMITS: use jacobian to translate contact forces to joint
%forces
tau = J' * t;
Constraints = [Constraints, abs(tau) <= s .* tau_max];

%CONTACT FORCE CONSTRAINTS: 

for i = 1:3
    % Get the 3x3 rotation matrix for contact i
    R_i = R_contacts(:,:,i);
    
    % Extract the 3x1 force vector for contact i from the 9x1 decision variable t
    f_global = t( (i-1)*3 + 1 : i*3 );
    
    % Transform the global force f_global to the local contact frame: f_local = R_i' * f_global
    f_local = R_i' * f_global;
    
    % The components in the local frame are:
    % f_local(1) = tangential_1 (t_i1 direction)
    % f_local(2) = tangential_2 (t_i2 direction)
    % f_local(3) = normal (n_i direction)
    
    % 1. FRICTION CONE CONSTRAINT: ||f_tangential|| <= mu * f_normal
    % f_tangential vector:
    f_tangential = f_local(1:2);
    
    % f_normal scalar:
    f_normal = f_local(3);
    
    Constraints = [Constraints, cone( f_tangential, mu * f_normal )];
    
    % 2. UNILATERAL CONSTRAINT: Normal force must be non-negative (pushes into object)
    % Since n_i points inward, f_normal >= 0.
    Constraints = [Constraints, f_normal >= 0];
end

Objective = s; % YALMIP minimizes by default

options = sdpsettings('verbose', 1, 'solver', 'mosek');
disp('Calling MOSEK to solve the LP...');
sol = optimize(Constraints, Objective, options);

if sol.problem == 0
    disp('Optimal solution found!');
    fprintf('\n--- Grasp Optimization Results ---\n');
    
    % Get the numerical values from the solver
    opt_tau = value(tau);
    opt_s = value(s);
    opt_t = value(t);
    
    fprintf('Optimal Cost Index (s): %.4f\n', opt_s);
    
    % --- I. Contact Force Table ---
    fprintf('\n## 1. Optimal Contact Forces (t) [N/mm]\n');
    
    % Reshape the 9x1 vector into a 3x3 matrix for easy viewing (3 components x 3 contacts)
    F_contacts = reshape(opt_t, 3, 3);
    
    % Use meaningful contact names
    ContactNames = {'Index Finger', 'Middle Finger', 'Thumb'};
    
    % Create a table for contact forces
    ContactForcesTable = table(...
        F_contacts(1, :)', ...
        F_contacts(2, :)', ...
        F_contacts(3, :)', ...
        'VariableNames', {'Fx_Global', 'Fy_Global', 'Fz_Global'}, ...
        'RowNames', ContactNames ...
    );
    
    disp(ContactForcesTable);
    
    % --- II. Joint Torque Table ---
    fprintf('\n## 2. Joint Torques (tau) and Utilization (k)\n');
    
    % The utilization ratio k_i = |tau_i| / tau_max_i
    k = abs(opt_tau) ./ tau_max;
    
    % The Allegro hand has 4 joints per finger, 4 fingers (Index, Middle, Ring, Thumb)
    % The current 'allegro_data.mat' likely only has 3 fingers (Index, Middle, Thumb) or a simplified model,
    % resulting in 16 joints (4*4). The labels below assume this order for 16 joints.
    JointLabels = {};
    Fingers = {'Index', 'Middle', 'Ring', 'Thumb'}; % Assuming 4 fingers for 16 joints
    JointTypes = {'J1', 'J2', 'J3', 'J4'}; % The four joints of each finger
    
    for f = 1:4
        for j = 1:4
            JointLabels{end+1} = sprintf('%s_%s', Fingers{f}, JointTypes{j});
        end
    end

    % Create a table for joint torques
    JointTorqueTable = table(...
        opt_tau, ...
        tau_max, ...
        k, ...
        'VariableNames', {'Opt_Torque_N_mm', 'Max_Torque_N_mm', 'Utilization_Ratio_k'}, ...
        'RowNames', JointLabels' ...
    );
    
    disp(JointTorqueTable);
    fprintf('Max Utilization Ratio (k_i): %.4f (This equals the optimal cost index s).\n', max(k));
    
    % --- III. Constraint Slack and Verification ---
    fprintf('\n## 3. Contact Constraint Slack and Verification\n');
    
    SlackTableData = zeros(3, 2);
    
    for i = 1:3
        % a. Normal Force (Unilateral) Constraint Slack
        % Constraint is: f_normal >= 0, or 0 - f_normal <= 0
        % Slack is: f_normal (if f_normal > 0, constraint is inactive)
        
        f_global = opt_t( (i-1)*3 + 1 : i*3 );
        f_local = R_contacts(:,:,i)' * f_global;
        f_normal = f_local(3);
        
        % For f_normal >= 0, the slack is the value of f_normal itself.
        % A value close to zero means the finger is barely touching (active/binding).
        Normal_Slack = f_normal;
        
        % b. Friction Cone Slack
        % Constraint is: ||f_tangential|| - mu * f_normal <= 0
        % Slack is: mu * f_normal - ||f_tangential||
        f_tangential = f_local(1:2);
        
        Friction_Demand = norm(f_tangential);
        Friction_Capacity = mu * f_normal;
        Friction_Slack = Friction_Capacity - Friction_Demand;
        
        % Store data
        SlackTableData(i, 1) = Normal_Slack;
        SlackTableData(i, 2) = Friction_Slack;
        
        % Optional: Verification for the active status of the cone
        % The dual variable for a cone constraint is a vector and is complex to display, 
        % so we stick to the primary slack variable for readability.
        % Dual for Friction Cone (YALMIP uses cone(t,v) which is L2(t) <= v)
        % slack is v - L2(t)
    end
    
    % Create a table for slack values
    SlackTable = table(...
        SlackTableData(:, 1), ...
        SlackTableData(:, 2), ...
        'VariableNames', {'Normal_Force_Slack', 'Friction_Cone_Slack'}, ...
        'RowNames', ContactNames ...
    );
    
    disp(SlackTable);
    fprintf(['\n* **Normal Force Slack:** Value of the normal force (f_normal). ',...
             'A value close to zero means the contact is barely touching (active).\n']);
    fprintf(['* **Friction Cone Slack:** Capacity (mu * f_normal) minus demand (||f_tangential||). ',...
             'A value close to zero means the contact is slipping or on the verge of slipping (active).\n']);
    
    % --- IV. Grasp Verification (Wrench check) ---
    fprintf('\n## 4. Wrench Equilibrium Verification\n');
    
    % Check how close the resultant wrench G*t is to the required external wrench -w_ext
    resultant_wrench = G * opt_t;
    required_wrench = -w_ext;
    
    fprintf('Required Wrench (-w_ext) [N/mm]:\n');
    disp(required_wrench');
    fprintf('Resultant Wrench (G*t) [N/mm]:\n');
    disp(resultant_wrench');
    
    % Display the error magnitude
    error_magnitude = norm(resultant_wrench - required_wrench);
    fprintf('Wrench Error Magnitude: %.4e (Should be close to zero, since it is an equality constraint)\n', error_magnitude);
    
else
    disp('Error: Problem could not be solved.');
    disp(sol.info);
end