%% 1. EXAMPLE 2. Two Fingered Grasp with a Hard Finger Contact Model

yalmip('clear');
clear;
clc;

w = [0; -1; 1; 0.5; 0; 0]; 
tau_max = [10; 10; 10; 10]; 

mu = 0.5;   %frictional coefficient
mu_t = 0.2; %proportional constant btwn the torsion and shear limits

M1 = [ 1/6   0     0     0     0     0;
       0    -1/3   0     0     0    -1/3;
       0     0    1/2    0    -1/2   0;
      -1/6   0     0     0     0     0;
       0    -1/3   0     0     0     1/3;
       0     0   -1/2    0    -1/2   0;
       0     0     0    1/2    0     0;
       0     0     0    1/2    0     0 ];

M2 = [ 5/12   0    -1/12   0;
       0    -1/3    0     0;
       0     0     0     0;
       1/12   0    -5/12   0;
       0     0     0     1/3;
       0     0     0     0;
       0     0     0     0;
       0     0     0     0 ];

tau = sdpvar(4,1);
s   = sdpvar(1,1);

t = M1*w + M2*tau;   % 8x1

% Interpret t according to the paper:
%normal --> x
%tangential --> y and z 
% t = [fix, fiy, fiz, miz]

f_norm1    = t(1);
f_tan1     = t(2:3);
m_torsion1 = t(4);

f_norm2    = t(5);
f_tan2     = t(6:7);
m_torsion2 = t(8);


Constraints = [];
Constraints = [Constraints, s >= 0];

% Joint-torque / cost-index constraints
Constraints = [Constraints, abs(tau) <= s .* tau_max];

% Normal forces (compressive)
Constraints = [Constraints, f_norm1 >= 0, f_norm2 >= 0];

% Soft-finger friction constraints:

% Contact 1: ||[fx1; fy1]||_2 <= mu * fz1
Constraints = [Constraints, norm(f_tan1,2) <= mu * f_norm1];

%           |mz1| <= mu_t * fz1
Constraints = [Constraints,  m_torsion1 <=  mu_t * f_norm1];
Constraints = [Constraints, -m_torsion1 <=  mu_t * f_norm1];

% Contact 2: ||[fx2; fy2]||_2 <= mu * fz2
Constraints = [Constraints, norm(f_tan2,2) <= mu * f_norm2];

%           |mz2| <= mu_t * fz2
Constraints = [Constraints,  m_torsion2 <=  mu_t * f_norm2];
Constraints = [Constraints, -m_torsion2 <=  mu_t * f_norm2];

Objective = s;

options = sdpsettings('verbose',1,'solver','mosek');
disp('Calling MOSEK to solve the SOCP...');
sol = optimize(Constraints, Objective, options);

if sol.problem == 0
    disp('Optimal solution found!');
    opt_tau = value(tau);
    opt_s   = value(s);

    fprintf('\n--- Results ---\n');
    fprintf('Optimal Cost Index (s): %.4f\n', opt_s);
    fprintf('Optimal Torques (tau):\n');
    disp(opt_tau);

    k = abs(opt_tau)./tau_max;
    fprintf('k_i = |tau_i| / tau_max_i:\n');
    disp(k);
    fprintf('max(k_i): %.4f\n', max(k));
else
    disp('Error: Problem could not be solved.');
    disp(sol.info);
end

