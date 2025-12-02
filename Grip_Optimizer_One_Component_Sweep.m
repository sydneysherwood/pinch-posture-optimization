% Grip_Optimizer_One_Component_Sweep_full.m
% Sweep each wrench component individually, enforce s <= 1, and compute
% relaxation-based infeasibility magnitude for infeasible samples.
%
% Requires: allegro_data.mat (Grasp_Matrix or G, Hand_Jacobian or J, R_contacts)
% Requires YALMIP and a solver supported by YALMIP (example uses 'mosek').

clear;
if exist('yalmip','file')==2
    yalmip('clear');
end
clc;

disp('Loading data...');
if exist('allegro_data.mat','file')~=2
    error('allegro_data.mat not found in current folder.');
end
S = load('allegro_data.mat');
% Accept multiple possible variable names
if isfield(S,'Grasp_Matrix'), G = S.Grasp_Matrix; elseif isfield(S,'G'), G = S.G; else error('Grasp_Matrix (or G) missing'); end
if isfield(S,'Hand_Jacobian'), J = S.Hand_Jacobian; elseif isfield(S,'J'), J = S.J; else error('Hand_Jacobian (or J) missing'); end
if isfield(S,'R_contacts'), R_contacts = S.R_contacts; else error('R_contacts missing'); end

% Baseline external wrench [Fx;Fy;Fz;Mx;My;Mz]
baseline = [0; 0; 0; 0; 0; 0];

% Joint torque limits (N·mm) -- example: 0.7 N·m = 700 N·mm
tau_max = repmat(0.7*1000, 16, 1); % adjust to your robot's DOF; column vector
nJ = numel(tau_max);

% Friction coefficient
mu = 0.86;

% Sweep settings
nPts = 101;
ranges = [
    -20   20;    % Fx
    -20   20;    % Fy
    -20   20;    % Fz
    -1000  1000;   % Mx
    -800  800;   % My
    -800  800;   % Mz
];

% Derived sizes
nContacts = size(R_contacts,3);
nT = 3 * nContacts;    % number of contact force variables (3 per contact)

% Ensure J orientation is consistent: desired tau_expr is nJ x 1 when multiplied by contact forces t (nT x 1).
% Accept either J sized [nT x nJ] (so tau = J' * t) or J sized [nJ x nT] (so tau = J * t).
if isequal(size(J), [nT, nJ])
    useJtranspose = true;
elseif isequal(size(J), [nJ, nT])
    useJtranspose = false;
else
    % Try transposing once if sizes are swapped accidentally
    if isequal(size(J'), [nT, nJ])
        J = J';
        useJtranspose = true;
    elseif isequal(size(J'), [nJ, nT])
        J = J';
        useJtranspose = false;
    else
        warning('Hand_Jacobian size [%d %d] unexpected. Expected [%d %d] or [%d %d]. Proceeding but errors may occur.', ...
            size(J,1), size(J,2), nT, nJ, nJ, nT);
        % best-effort: assume J is nT x nJ
        useJtranspose = true;
    end
end

% Storage
samples    = zeros(nPts,6,6);
isFeas     = false(nPts,6);
s_vals     = nan(nPts,6);
r_vals     = nan(nPts,6);
tau_vals   = cell(nPts,6);          % store joint torques if feasible
infeas_mag = nan(nPts,6);           % relaxation-based infeasibility magnitude
solveInfo  = cell(nPts,6);

% Solver settings (use mosek if available, otherwise use default)
if exist('yalmip','file')==2
    solverName = 'mosek';
    if isempty(which(solverName))
        solverName = ''; % let YALMIP pick default
    end
    options = sdpsettings('verbose',0,'solver',solverName);
else
    error('YALMIP not found. This script requires YALMIP.');
end

lambda = 0;
active_joint_indices = [1:8, 13:16]; % adjust per your robot
active_joint_indices = active_joint_indices(active_joint_indices <= nJ);
tol = 1e-6;                           % numerical tolerance for s check

disp('Starting sweeps (one component at a time)...');
for comp = 1:6
    vals = linspace(ranges(comp,1), ranges(comp,2), nPts);
    for k = 1:nPts
        % sample wrench
        w_ext_k = baseline;
        w_ext_k(comp) = vals(k);
        samples(k,:,comp) = w_ext_k';

        % decision variables
        s = sdpvar(1,1);
        t = sdpvar(nT,1);       % contact forces
        k_min = sdpvar(1,1);
        r = sdpvar(1,1);

        % build constraints
        C = [];
        % equilibrium
        C = [C, G * t == -w_ext_k];

        % joint torques and utilizations with robust orientation handling
        if useJtranspose
            tau_expr = J' * t;   % J' is nJ x nT
        else
            tau_expr = J * t;    % J is nJ x nT
        end
        % ensure column shape
        tau_expr = reshape(tau_expr, nJ, 1);

        k_all = abs(tau_expr) ./ tau_max;  % nJ x 1
        C = [C, k_all <= s];
        if ~isempty(active_joint_indices)
            C = [C, k_all(active_joint_indices) >= k_min];
        else
            C = [C, k_min <= s];
        end
        C = [C, r == s - k_min];
        % enforce s physical cap
        C = [C, s <= 1];

        % contact constraints
        for iC = 1:nContacts
            R_i = R_contacts(:,:,iC);
            f_global = t((iC-1)*3 + 1 : iC*3);
            f_local = R_i' * f_global;
            f_tang = f_local(1:2);
            f_norm = f_local(3);
            C = [C, cone(f_tang, mu * f_norm)];
            C = [C, f_norm >= 0];
        end

        % objective: minimize s (lambda = 0 typical)
        Obj = (1-lambda)*s + lambda*r;

        % solve nominal
        sol = optimize(C, Obj, options);
        solveInfo{k,comp} = sol;

        % decide feasibility
        if sol.problem == 0
            s_val = value(s);
            if isempty(s_val) || s_val > 1 + tol
                isFeas(k,comp) = false;
                s_vals(k,comp) = NaN;
                r_vals(k,comp) = NaN;
                tau_vals{k,comp} = [];
            else
                isFeas(k,comp) = true;
                s_vals(k,comp) = s_val;
                r_vals(k,comp) = value(r);
                % attempt to retrieve tau values safely
                tev = value(tau_expr);
                if ~isempty(tev)
                    tau_vals{k,comp} = tev(:);
                else
                    tau_vals{k,comp} = [];
                end
            end
        else
            isFeas(k,comp) = false;
            s_vals(k,comp) = NaN;
            r_vals(k,comp) = NaN;
            tau_vals{k,comp} = [];
        end

        % If infeasible, run a relaxation to quantify infeasibility magnitude
        if ~isFeas(k,comp)
            % Build relaxed problem: allow equilibrium slack and utilization slack
            m_eq = size(G,1);
            slack_eq = sdpvar(m_eq,1);
            slack_k  = sdpvar(nJ,1);
            tR = sdpvar(nT,1);
            sR = sdpvar(1,1);
            k_minR = sdpvar(1,1);
            rR = sdpvar(1,1);

            Crel = [];
            % relaxed equilibrium: G*tR + slack_eq == -w_ext_k
            Crel = [Crel, G*tR + slack_eq == -w_ext_k];
            % utilization with slack_k >= 0
            if useJtranspose
                tau_exprR = J' * tR;
            else
                tau_exprR = J * tR;
            end
            tau_exprR = reshape(tau_exprR, nJ, 1);
            k_allR = abs(tau_exprR) ./ tau_max;
            Crel = [Crel, k_allR <= sR + slack_k];
            Crel = [Crel, slack_k >= 0];
            % contact cone constraints (not relaxed here)
            for iC = 1:nContacts
                R_i = R_contacts(:,:,iC);
                f_globalR = tR((iC-1)*3 + 1 : iC*3);
                f_localR = R_i' * f_globalR;
                f_tangR = f_localR(1:2);
                f_normR = f_localR(3);
                Crel = [Crel, cone(f_tangR, mu * f_normR)];
                Crel = [Crel, f_normR >= 0];
            end
            % keep s cap and other relations
            Crel = [Crel, sR <= 1];
            if ~isempty(active_joint_indices)
                Crel = [Crel, k_allR(active_joint_indices) >= k_minR];
            else
                Crel = [Crel, k_minR <= sR];
            end
            Crel = [Crel, rR == sR - k_minR];

            % Objective: minimize 1-norm of slack_eq and sum of slack_k
            u = sdpvar(m_eq,1);
            Crel = [Crel, u >= slack_eq, u >= -slack_eq, u >= 0];
            ObjRel = sum(u) + sum(slack_k);

            solRel = optimize(Crel, ObjRel, options);
            if solRel.problem == 0
                infeas_mag(k,comp) = value(ObjRel);
            else
                infeas_mag(k,comp) = Inf;
            end
        else
            infeas_mag(k,comp) = 0;
        end
    end
    fprintf('Component %d sweep done: feasible %d / %d\n', comp, sum(isFeas(:,comp)), nPts);
end

% Save results
save('grasp_sweep_results_with_relax.mat', 'samples', 'isFeas', 's_vals', 'r_vals', ...
    'tau_vals', 'infeas_mag', 'solveInfo', 'ranges', 'baseline');

% --- Plot s only, mark infeasible samples (infeasible if solver failed OR s>1) ---
compNames = {'Fx','Fy','Fz','Mx','My','Mz'};
figure('Name','s vs Sweep (feasibility with s<=1)','NumberTitle','off','Units','normalized','Position',[0.05 0.05 0.9 0.85]);

% Global finite s max for marker placement
finite_s = s_vals(~isnan(s_vals));
if isempty(finite_s)
    global_s_max = 1;
else
    global_s_max = max(finite_s);
end
y_infeas_level_global = global_s_max * 1.05 + 1e-6;

for comp = 1:6
    x = squeeze(samples(:,comp,comp));    % sweep values for this component
    if ~isvector(x), x = x(:); end
    y = s_vals(:,comp);                   % s (NaN for infeasible)
    subplot(3,2,comp);
    hold on; box on; grid on;
    plot(x, y, '-g', 'LineWidth', 1.5); % green line (NaNs break line)
    if any(isFeas(:,comp))
        plot(x(isFeas(:,comp)), y(isFeas(:,comp)), 'go', 'MarkerSize', 4, 'MarkerFaceColor', 'g');
    end

    infeas_idx = find(~isFeas(:,comp));
    y_infeas_level = y_infeas_level_global; % default fallback
    if ~isempty(infeas_idx)
        if all(isnan(y))
            y_infeas_level = y_infeas_level_global;
        else
            s_comp_max = max(y(~isnan(y)));
            y_infeas_level = s_comp_max * 1.05 + 1e-6;
        end
        plot(x(infeas_idx), repmat(y_infeas_level, numel(infeas_idx), 1), 'rx', 'MarkerSize', 8, 'LineWidth',1.2);

        % Annotate up to 3 infeasible points robustly
        nAnnot = min(3, numel(infeas_idx));
        for jj = 1:nAnnot
            i = infeas_idx(jj);
            xt = x(i);
            yt = y_infeas_level;
            val = infeas_mag(i,comp);
            if isfinite(val)
                txt = sprintf('%.2g', val);
            else
                txt = 'Inf';
            end
            text(xt, yt, [' ' txt], 'Color','r', 'FontSize',8);
        end
    end

    % Adjust y limits safely
    if all(isnan(y))
        ylim_lower = 0;
    else
        ylim_lower = min(y(~isnan(y)), [], 'omitnan');
    end
    if isempty(ylim_lower); ylim_lower = 0; end
    ylim_upper = max([y(~isnan(y)); y_infeas_level_global], [], 'omitnan');
    if isempty(ylim_upper); ylim_upper = y_infeas_level_global; end
    ylim([ylim_lower - 0.05*abs(ylim_lower+1), ylim_upper * 1.1]);

    xlabel(compNames{comp});
    ylabel('s');
    title(sprintf('Sweep of %s (others at baseline)', compNames{comp}));
    hold off;
end

% --- Optional: Objective-function plot (s and relaxation) ---
figure('Name','Objective vs Sweep','NumberTitle','off','Units','normalized','Position',[0.05 0.05 0.9 0.85]);
for comp = 1:6
    vals = squeeze(samples(:,comp,comp)); if ~isvector(vals), vals = vals(:); end
    svals = s_vals(:,comp);
    relax = infeas_mag(:,comp);

    subplot(3,2,comp); hold on; grid on; box on;
    yyaxis left
    plot(vals, svals, '-g', 'LineWidth',1.5);
    plot(vals(isFeas(:,comp)), svals(isFeas(:,comp)), 'go','MarkerFaceColor','g');
    ylabel('s (nominal objective)');
    yline(1, '--k', 's = 1');

    yyaxis right
    infeas_idx = find(~isFeas(:,comp));
    if ~isempty(infeas_idx)
        plot(vals(infeas_idx), relax(infeas_idx), 'rx', 'MarkerSize',8, 'LineWidth',1.2);
    end
    ylabel('Relaxation objective (infeas\_mag)');
    title(sprintf('Objective vs %s', compNames{comp}));
    xlabel(sprintf('%s sweep value', compNames{comp}));
    legend({'s (nominal)','feasible points','s = 1','relaxation (infeasible)'}, 'Location','best');
    hold off;
end

% Feasibility summary
fprintf('\nFeasibility summary per component (treat s>1 as infeasible):\n');
for comp = 1:6
    fprintf('  %s: feasible %d / %d (%.1f%%)\n', compNames{comp}, sum(isFeas(:,comp)), nPts, 100*sum(isFeas(:,comp))/nPts);
end

disp('Done. Results saved to grasp_sweep_results_with_relax.mat');
