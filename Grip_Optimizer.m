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
mu = 1;

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

for i = 1:1
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
    fprintf('\n--- Results ---\n');
    
    % Get the numerical values from the solver
    opt_tau = value(tau);
    opt_s = value(s);
    opt_t = value(t)
    
    fprintf('Optimal Cost Index (s): %.4f\n', opt_s);
    fprintf('Optimal Torques (tau) [IN N/MM]:\n');
    disp(opt_tau);

    fprintf('Optimal Contact Forces:\n');
    disp(opt_t)
    
    % --- Verification ---
    fprintf('\n--- Verification ---\n');
    k = abs(opt_tau) ./ tau_max;
    fprintf('k_i = |tau_i| / tau_max_i:\n');
    disp(k);
    fprintf('max(k_i): %.4f\n', max(k));
    
else
    disp('Error: Problem could not be solved.');
    disp(sol.info);
end

%above code matches the paper output for EXAMPLE 1 !!

