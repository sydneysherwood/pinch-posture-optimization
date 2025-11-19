yalmip('clear');
clear;
clc;



%% 1. EXAMPLE 1. Two Fingered Grasp with a Hard Finger Contact Model
disp('Setting up the optimization problem for Example 1...');
% These values are taken directly from Example 1 (Page 5-6)

% External wrench [Fx, Fy, Mz]' applied *on* the object
w = [0; -1; 0]; 
% Joint torque limits [tau1_max, tau2_max, tau3_max]' [ABSOLUTE VALUE]
tau_max = [5; 3; 5]; 
% Friction coefficient (mu)
mu = 0.5;

% M1 and M2 matrices that map (w, tau) to contact forces t
% t = M1*w + M2*tau
% t = [fx1, fy1, fx2, fy2, fx3, fy3]'

M1 = [ -2   0    0;
      -0.5  -0.5  0.5;
      -1    0    0;
       0      0     0;
       0      0     0;
       0.5  -0.5 -0.5 ]; 

M2 = [ -1    0    -2;
      -0.5  0.5  -0.5;
      -1    0    -1;
       0    1    0;  % This is t(4) = fy2
       0    0    -1; % This is t(5) = fx3
       0.5  0.5  0.5 ]; % This is t(6) = fy3

%DECISION VARIABLES

%sdpvar --> symbolic decision variables
%inputs are length, height --> e.g. 1,1 is a scalar var

% tau: The (3x1) vector of joint torques we are solving for
tau = sdpvar(3, 1); 

% s: The scalar "maximal cost index" we want to minimize
s = sdpvar(1, 1); 

%DEFINE FORCES
% Define the contact force vector 't' as a linear expression of 'tau'.
% YALMIP will automatically substitute this into the constraints.
t = M1*w + M2*tau; % 

fx1 = t(1); % Contact 1 Normal
fy1 = t(2); % Contact 1 Tangent
fx2 = t(3); % Contact 2 Tangent
fy2 = t(4); % Contact 2 Normal
fx3 = t(5); % Contact 3 Normal
fy3 = t(6); % Contact 3 Tangent

%DEFINE CONSTRAINTS
Constraints = [];

% --- Objective/Joint Limit Constraints: ---
% |tau_i| <= s * tau_max_i --> can just input directly as vectors
Constraints = [Constraints, abs(tau) <= s .* tau_max];

% --- Friction Constraints ---
% |f_tangent| <= mu * f_normal
Constraints = [Constraints, abs(fy1) <= mu * fx1]; % Contact 1
Constraints = [Constraints, abs(fx2) <= mu * fy2]; % Contact 2
Constraints = [Constraints, abs(fy3) <= mu * fx3]; % Contact 3

% --- Normal Force Constraints ---
% Normal forces must be positive (pushing into the object)
Constraints = [Constraints, fx1 >= 0];
Constraints = [Constraints, fy2 >= 0];
Constraints = [Constraints, fx3 >= 0];

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
    
    fprintf('Optimal Cost Index (s): %.4f\n', opt_s);
    fprintf('Optimal Torques (tau):\n');
    disp(opt_tau);
    
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


