%% PIANIFICAZIONE E CONTROLLO DI UN ROBOT MOBILE DIFFERENZIALE
% Universita' San Raffaele Roma - Corso di Robotica
% Script principale completo:
%   1) Lettura configurazione centralizzata (home, intermedi, traguardo, dimensioni, SVG)
%   2) Pianificazione multi-tratta (Cell Decomposition + Retraction + Dijkstra)
%   3) Modalita' parametrica PERCORSI INTELLIGENTI (taglio punti intermedi e curve morbide)
%   4) Orientamento preliminare della prua e transito anti-slittamento a velocita' costante
%   5) Integrazione numerica RK4, telemetria live, grafici ad alta risoluzione e animazione 2D

clear; clc; close all;

%% 1. Inizializzazione ambiente, configurazione percorso e parametri del robot
[env, robot] = crea_ambiente();

% Gestione sicura dei campi opzionali di configurazione
if ~isfield(env, 'goals') || isempty(env.goals)
env.goals = env.qf(1:2)';
end
N_goals = size(env.goals, 1);

if ~isfield(env, 'goal_labels') || numel(env.goal_labels) ~= N_goals
env.goal_labels = cell(N_goals, 1);
for g = 1:(N_goals - 1)
env.goal_labels{g} = sprintf('P%d', g);
end
env.goal_labels{N_goals} = 'TRAGUARDO';
end

if ~isfield(env, 'goal_is_home') || numel(env.goal_is_home) ~= N_goals
env.goal_is_home = false(N_goals, 1);
end

if ~isfield(env, 'percorsi_intelligenti')
env.percorsi_intelligenti = true;
end
if ~isfield(env, 'soglia_risparmio_smart')
env.soglia_risparmio_smart = 0.15;
end
if ~isfield(env, 'angolo_no_decel_deg')
env.angolo_no_decel_deg = 45.0;
end
if ~isfield(env, 'acc_lat_max')
env.acc_lat_max = 1.25;
end
if ~isfield(env, 'fattore_velocita')
env.fattore_velocita = 1.0;
end

if env.percorsi_intelligenti
stato_smart_str = 'ATTIVO';
else
stato_smart_str = 'DISATTIVO';
end

fprintf('=== AVVIO SIMULAZIONE ROBOT MOBILE DIFFERENZIALE ===\n');
fprintf('Workspace: [%.1f, %.1f] x [%.1f, %.1f] m\n', ...
env.xlim(1), env.xlim(2), env.ylim(1), env.ylim(2));
fprintf('Robot: r = %.3f m | asse d = %.3f m | R_body = %.3f m | R_safe = %.3f m\n', ...
robot.r, robot.d, robot.R_body, robot.R_safe);
fprintf('PERCORSO INTELLIGENTE: %s\n', stato_smart_str);
fprintf('HOME q0 = [%.2f, %.2f, %.2f rad] | Tappe configurate: %d\n', ...
env.q0(1), env.q0(2), env.q0(3), N_goals);
for g = 1:N_goals
fprintf('  -> Tappa %d (%s): [%.2f, %.2f] m\n', ...
g, env.goal_labels{g}, env.goals(g, 1), env.goals(g, 2));
end

%% 2. Pianificazione nominale e ottimizzazione con Percorsi Intelligenti
tic;

% 2a. Pianificazione nominale di riferimento su tutte le tappe
legs_nom = cell(N_goals, 1);
p_curr = env.q0(1:2)';
grid_data = [];
for g = 1:N_goals
p_next = env.goals(g, :);
[wp_g, gd_g] = pianifica_celle_retraction(env, robot, p_curr, p_next);
legs_nom{g}.waypoints = wp_g;
legs_nom{g}.raw_path  = gd_g.raw_path;
legs_nom{g}.lunghezza = sum(hypot(diff(wp_g(:, 1)), diff(wp_g(:, 2))));
if g == 1
grid_data = gd_g;
end
p_curr = p_next;
end

lunghezza_nominale = 0.0;
for g = 1:N_goals
lunghezza_nominale = lunghezza_nominale + legs_nom{g}.lunghezza;
end

% 2b. Applicazione eventuale della modalita' Percorsi Intelligenti
goal_skipped = false(N_goals, 1);
if env.percorsi_intelligenti
lambda_smart = 0.015;
if isfield(robot, 'R_safe_smart') && ~isempty(robot.R_safe_smart)
r_safe_smart = robot.R_safe_smart;
else
r_safe_smart = robot.R_body + 0.06;
end

legs = {};
leg_goal_idx = [];
p_curr = env.q0(1:2)';
g = 1;

while g <= N_goals
    % Valutazione salto punto intermedio (esclusi i ritorni a HOME e il TRAGUARDO)
    if g < N_goals && ~env.goal_is_home(g)
        wp_a = legs_nom{g}.waypoints;
        wp_b = legs_nom{g + 1}.waypoints;
        L_via = sum(hypot(diff(wp_a(:, 1)), diff(wp_a(:, 2)))) + ...
                sum(hypot(diff(wp_b(:, 1)), diff(wp_b(:, 2))));

        L_dir = inf;
        wp_dir = [];
        gd_dir = [];
        try
            [wp_dir_raw, gd_dir] = pianifica_celle_retraction( ...
                env, robot, p_curr, env.goals(g + 1, :), r_safe_smart, lambda_smart);
            wp_dir = ottimizza_smart_curve(wp_dir_raw, env, r_safe_smart);
            L_dir = sum(hypot(diff(wp_dir(:, 1)), diff(wp_dir(:, 2))));
        catch
            L_dir = inf;
        end

        if L_dir + env.soglia_risparmio_smart < L_via
            goal_skipped(g) = true;
            fprintf('  [Percorso Intelligente] Tappa %s saltata: tratto diretto %.2f m vs %.2f m (-%.2f m).\n', ...
                env.goal_labels{g}, L_dir, L_via, L_via - L_dir);
            leg_new.waypoints = wp_dir;
            leg_new.raw_path  = gd_dir.raw_path;
            leg_new.lunghezza = L_dir;
            legs{end + 1} = leg_new; %#ok<AGROW>
            leg_goal_idx(end + 1) = g + 1; %#ok<AGROW>
            p_curr = env.goals(g + 1, :);
            g = g + 2;
            continue;
        end
    end

    % Mantenimento tappa g con curvatura morbida e margine smart sicuro
    [wp_sm_raw, gd_sm] = pianifica_celle_retraction( ...
        env, robot, p_curr, env.goals(g, :), r_safe_smart, lambda_smart);
    wp_sm = ottimizza_smart_curve(wp_sm_raw, env, r_safe_smart);
    leg_new.waypoints = wp_sm;
    leg_new.raw_path  = gd_sm.raw_path;
    leg_new.lunghezza = sum(hypot(diff(wp_sm(:, 1)), diff(wp_sm(:, 2))));
    legs{end + 1} = leg_new; %#ok<AGROW>
    leg_goal_idx(end + 1) = g; %#ok<AGROW>
    p_curr = env.goals(g, :);
    g = g + 1;
end


else
legs = legs_nom;
leg_goal_idx = 1:N_goals;
end
t_plan = toc;

N_legs = numel(legs);

% Concatenazione dei waypoint attivi e del percorso nominale per grafici e tracking
all_waypoints = legs{1}.waypoints;
all_raw_path  = legs{1}.raw_path;
for l = 2:N_legs
all_waypoints = [all_waypoints; legs{l}.waypoints(2:end, :)]; %#ok
all_raw_path  = [all_raw_path;  legs{l}.raw_path(2:end, :)];  %#ok
end

nom_waypoints = legs_nom{1}.waypoints;
for g = 2:N_goals
nom_waypoints = [nom_waypoints; legs_nom{g}.waypoints(2:end, :)]; %#ok
end

M_tot             = size(all_waypoints, 1);
n_free            = nnz(grid_data.free_mask);
n_tot             = numel(grid_data.free_mask);
lunghezza_plan    = sum(hypot(diff(all_waypoints(:, 1)), diff(all_waypoints(:, 2))));
metri_risparmiati = max(0.0, lunghezza_nominale - lunghezza_plan);

fprintf('Pianificazione completata in %.3f s: %d/%d celle libere (%.1f%%).\n', ...
t_plan, n_free, n_tot, 100 * n_free / n_tot);
fprintf('Waypoint totali: %d | Lunghezza nominale: %.3f m | Pianificata finale: %.3f m\n', ...
M_tot, lunghezza_nominale, lunghezza_plan);
if env.percorsi_intelligenti
fprintf('Risparmio stimato da Percorso Intelligente: %.3f m\n', metri_risparmiati);
end

%% 3. Impostazione velocita', controllore cinematico e simulazione RK4
fattore_velocita = env.fattore_velocita;
v_crociera       = 0.75 * fattore_velocita; % Velocita' di crociera impostata [m/s]
adatta_motori    = true;                    % Adegua omega_max se v_crociera > r*omega_max

ctrl.v_max   = v_crociera;                  % Tetto velocita' lineare [m/s]
ctrl.k_rho   = 1.30 * fattore_velocita;     % Guadagno lineare [1/s]
ctrl.k_alpha = 3.60 * fattore_velocita;     % Guadagno angolare [1/s] (k_alpha > k_rho)
ctrl.r       = robot.r;
ctrl.d       = robot.d;

v_fisico_max = robot.r * robot.omega_max;
if adatta_motori && (v_crociera > 0.85 * v_fisico_max)
ctrl.omega_max = (v_crociera * 1.35) / robot.r;
else
ctrl.omega_max = robot.omega_max;
end

fprintf('Velocita'' crociera impostata: %.2f m/s | Limite motori attivo: %.2f rad/s\n', ...
ctrl.v_max, ctrl.omega_max);

dt           = 0.02;                        % Passo di integrazione [s] (50 Hz)
T_max        = max(140.0, 65.0 * N_legs / max(fattore_velocita, 0.25));
N_max        = round(T_max / dt);
r_switch     = 0.26;                        % Raggio commutazione waypoint intermedi [m]
r_switch_int = 0.30;                        % Raggio transito fluido su tappe intermedie [m]
tol_goal     = 0.03;                        % Tolleranza arresto al traguardo finale [m]
tol_align    = 1.5 * pi / 180;              % Tolleranza orientamento preliminare prua [rad]

% Preallocazione vettori telemetria
t_log         = zeros(N_max, 1);
q_log         = zeros(N_max, 3);
wR_log        = zeros(N_max, 1);
wL_log        = zeros(N_max, 1);
v_log         = zeros(N_max, 1);
w_log         = zeros(N_max, 1);
s_log         = zeros(N_max, 1);
vavg_log      = zeros(N_max, 1);
rho_log       = zeros(N_max, 1);
alpha_log     = zeros(N_max, 1);
ect_log       = zeros(N_max, 1);
dmin_log      = zeros(N_max, 1);
dist_goal_log = zeros(N_max, 1);
leg_log       = zeros(N_max, 1);
align_log     = false(N_max, 1);

q = env.q0;
leg_idx = 1;
wp_leg  = legs{leg_idx}.waypoints;
M_leg   = size(wp_leg, 1);
idx_wp  = min(2, M_leg);
fase_allineamento = true; % Prima azione: orientare la prua verso la traiettoria

k_step = 0;
s_percorsa = 0.0;
collisione_rilevata = false;

for k = 1:N_max
t = (k - 1) * dt;

% Verifica fase di orientamento preliminare della prua
if fase_allineamento
    wp_target = wp_leg(idx_wp, :)';
    dx_al = wp_target(1) - q(1);
    dy_al = wp_target(2) - q(2);
    phi_al = atan2(dy_al, dx_al);
    alpha_al = atan2(sin(phi_al - q(3)), cos(phi_al - q(3)));

    if abs(alpha_al) <= tol_align
        fase_allineamento = false;
        fprintf('  -> Tratta %d/%d allineata a t = %.2f s (theta = %.1f deg): avvio avanzamento.\n', ...
            leg_idx, N_legs, t, q(3) * 180 / pi);
    end
end

% Gestione waypoint target e logica anti-slittamento per transito costante
if fase_allineamento
    wp_target = wp_leg(idx_wp, :)';
    ctrl.solo_orientamento = true;
    ctrl.mantieni_velocita = false;
else
    dist_wp = hypot(wp_leg(idx_wp, 1) - q(1), wp_leg(idx_wp, 2) - q(2));
    if idx_wp < M_leg && dist_wp < r_switch
        idx_wp = idx_wp + 1;
    end
    wp_target = wp_leg(idx_wp, :)';
    ctrl.solo_orientamento = false;

    % Verifica se e' possibile mantenere la velocita' costante senza slittamenti
    if idx_wp < M_leg
        p_prev = q(1:2)';
        p_curr_wp = wp_leg(idx_wp, :);
        p_next_wp = wp_leg(idx_wp + 1, :);
        ctrl.mantieni_velocita = valuta_transito_fluido( ...
            p_prev, p_curr_wp, p_next_wp, ctrl.v_max, r_switch, ...
            env.angolo_no_decel_deg, env.acc_lat_max);
    elseif leg_idx < N_legs
        % Tappa intermedia della missione: valuta angolo tra tratta entrante e uscente
        p_prev_seg = wp_leg(max(1, M_leg - 1), :);
        p_node_int = wp_leg(M_leg, :);
        wp_next_leg = legs{leg_idx + 1}.waypoints;
        p_next_seg = wp_next_leg(min(2, size(wp_next_leg, 1)), :);
        ctrl.mantieni_velocita = valuta_transito_fluido( ...
            p_prev_seg, p_node_int, p_next_seg, ctrl.v_max, r_switch_int, ...
            env.angolo_no_decel_deg, env.acc_lat_max);
    else
        % Approccio finale al TRAGUARDO: decelerazione proporzionale v = k_rho * rho
        ctrl.mantieni_velocita = false;
    end
end

[wR, wL, ~, ~, rho, alpha] = controllore_differenziale(q, wp_target, ctrl);

% Velocita' effettive applicate dal modello differenziale
v_eff = (robot.r / 2.0) * (wR + wL);
w_eff = (robot.r / robot.d) * (wR - wL);

% Calcolo errore cross-track, clearance e distanza dal target corrente della tratta
e_ct = calcola_cross_track(q(1), q(2), all_waypoints);
d_obs_now = calcola_distanza_ostacoli(q(1), q(2), env);
clearance_now = d_obs_now - robot.R_body;
goal_curr_idx = leg_goal_idx(leg_idx);
dist_goal_now = hypot(q(1) - env.goals(goal_curr_idx, 1), q(2) - env.goals(goal_curr_idx, 2));

% Salvataggio log
k_step = k;
t_log(k)         = t;
q_log(k, :)      = q';
wR_log(k)        = wR;
wL_log(k)        = wL;
v_log(k)         = v_eff;
w_log(k)         = w_eff;
s_log(k)         = s_percorsa;
if t > 0
    vavg_log(k) = s_percorsa / t;
else
    vavg_log(k) = v_eff;
end
rho_log(k)       = rho;
alpha_log(k)     = alpha;
ect_log(k)       = e_ct;
dmin_log(k)      = clearance_now;
dist_goal_log(k) = dist_goal_now;
leg_log(k)       = leg_idx;
align_log(k)     = fase_allineamento;

% Verifica anticollisione
if clearance_now <= 0
    warning('Collisione rilevata nella tratta %d a t = %.2f s (clearance = %.4f m)!', ...
        leg_idx, t, clearance_now);
    collisione_rilevata = true;
    break;
end

% Transizione verso tappa successiva o arresto al traguardo finale
if ~fase_allineamento && idx_wp == M_leg
    if leg_idx < N_legs
        % Controllo transito fluido vs rallentamento sul punto intermedio
        p_prev_seg = wp_leg(max(1, M_leg - 1), :);
        p_node_int = wp_leg(M_leg, :);
        wp_next_leg = legs{leg_idx + 1}.waypoints;
        p_next_seg = wp_next_leg(min(2, size(wp_next_leg, 1)), :);
        [transito_ok, dpsi_deg, a_c_est] = valuta_transito_fluido( ...
            p_prev_seg, p_node_int, p_next_seg, ctrl.v_max, r_switch_int, ...
            env.angolo_no_decel_deg, env.acc_lat_max);

        if transito_ok
            soglia_passaggio = r_switch_int;
        else
            soglia_passaggio = tol_goal;
        end

        if rho < soglia_passaggio
            if transito_ok
                fprintf('  -> Tappa %s superata SENZA decelerazione (dpsi = %.1f deg, a_c = %.2f m/s^2) a t = %.2f s\n', ...
                    env.goal_labels{goal_curr_idx}, dpsi_deg, a_c_est, t);
                fase_allineamento = false;
            else
                fprintf('  -> Tappa %s raggiunta con rallentamento anti-slittamento (dpsi = %.1f deg) a t = %.2f s\n', ...
                    env.goal_labels{goal_curr_idx}, dpsi_deg, t);
                fase_allineamento = true;
            end
            leg_idx = leg_idx + 1;
            wp_leg  = legs{leg_idx}.waypoints;
            M_leg   = size(wp_leg, 1);
            idx_wp  = min(2, M_leg);
        end
    else
        if rho < tol_goal
            fprintf('  -> %s [%.2f, %.2f] raggiunto a t = %.2f s (metri percorsi: %.2f m)\n', ...
                env.goal_labels{goal_curr_idx}, env.goals(goal_curr_idx, 1), ...
                env.goals(goal_curr_idx, 2), t, s_percorsa);
            break;
        end
    end
end

% Integrazione numerica Runge-Kutta 4 (Zero-Order Hold su wR, wL)
f_kin = @(st) [v_eff * cos(st(3)); v_eff * sin(st(3)); w_eff];
k1 = f_kin(q);
k2 = f_kin(q + 0.5 * dt * k1);
k3 = f_kin(q + 0.5 * dt * k2);
k4 = f_kin(q + dt * k3);
q_next = q + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
q_next(3) = atan2(sin(q_next(3)), cos(q_next(3)));

% Aggiornamento metri percorsi istantanei
ds = hypot(q_next(1) - q(1), q_next(2) - q(2));
s_percorsa = s_percorsa + ds;
q = q_next;


end

% Taglio array alla lunghezza effettiva
t_log         = t_log(1:k_step);
q_log         = q_log(1:k_step, :);
wR_log        = wR_log(1:k_step);
wL_log        = wL_log(1:k_step);
v_log         = v_log(1:k_step);
w_log         = w_log(1:k_step);
s_log         = s_log(1:k_step);
vavg_log      = vavg_log(1:k_step);
rho_log       = rho_log(1:k_step);
alpha_log     = alpha_log(1:k_step);
ect_log       = ect_log(1:k_step);
dmin_log      = dmin_log(1:k_step);
dist_goal_log = dist_goal_log(1:k_step);
leg_log       = leg_log(1:k_step);
align_log     = align_log(1:k_step);

%% 4. Calcolo indici di prestazione
lunghezza_exec = s_log(end);
if t_log(end) > 0
vel_media_tot = lunghezza_exec / t_log(end);
else
vel_media_tot = 0.0;
end
vel_max_raggiunta = max(v_log);
rmse_ct           = sqrt(mean(ect_log.^2));
max_ct            = max(ect_log);
mean_ct           = mean(ect_log);
min_clearance     = min(dmin_log);
err_finale        = hypot(q_log(end, 1) - env.goals(end, 1), q_log(end, 2) - env.goals(end, 2));

fprintf('\n--- RIEPILOGO PRESTAZIONI SIMULAZIONE (per Tabella 3 LaTeX) ---\n');
fprintf('Modalita'' Percorso Intelligente : %s\n', stato_smart_str);
fprintf('Rapporto celle libere / totali  : %d / %d (%.1f%%)\n', n_free, n_tot, 100 * n_free / n_tot);
fprintf('Lunghezza nominale / finale     : %.3f m / %.3f m (risparmio: %.3f m)\n', ...
lunghezza_nominale, lunghezza_plan, metri_risparmiati);
fprintf('Metri percorsi totali (L_exec)  : %.3f m\n', lunghezza_exec);
fprintf('Durata missione T_tot           : %.2f s\n', t_log(end));
fprintf('Velocita'' crociera impostata    : %.3f m/s\n', ctrl.v_max);
fprintf('Velocita'' media sul percorso    : %.3f m/s (picco: %.3f m/s)\n', vel_media_tot, vel_max_raggiunta);
fprintf('Clearance minima dagli ostacoli : %.3f m\n', min_clearance);
fprintf('Errore cross-track medio / RMSE : %.4f m / %.4f m\n', mean_ct, rmse_ct);
fprintf('Errore cross-track massimo      : %.4f m\n', max_ct);
fprintf('Errore sull''ultimo traguardo    : %.4f m\n', err_finale);
fprintf('Collisione rilevata             : %d\n', collisione_rilevata);

%% 5. Visualizzazione Risultati Statici ed Esportazione PNG (300 dpi)

% Figura 1: Decomposizione in celle, Retraction e confronto nominale/intelligente
fig1 = figure('Name', '1 - Cell Decomposition e Retraction', 'Color', 'w', 'Position', [80 120 780 560]);
hold on; grid on; box on;
imagesc(grid_data.xc, grid_data.yc, double(~grid_data.free_mask));
colormap([1 1 1; 0.88 0.90 0.94]);
set(gca, 'YDir', 'normal');
disegna_ostacoli(env);
plot(grid_data.X(grid_data.skel_mask), grid_data.Y(grid_data.skel_mask), ...
'.', 'Color', [0.15 0.55 0.75], 'MarkerSize', 6, 'DisplayName', 'Scheletro Retraction');
if env.percorsi_intelligenti
plot(nom_waypoints(:, 1), nom_waypoints(:, 2), ':', 'Color', [0.55 0.55 0.55], ...
'LineWidth', 1.6, 'DisplayName', 'Percorso nominale');
end
plot(all_raw_path(:, 1), all_raw_path(:, 2), ...
'--', 'Color', [0.90 0.45 0.10], 'LineWidth', 1.3, 'DisplayName', 'Percorso Dijkstra grezzo');
plot(all_waypoints(:, 1), all_waypoints(:, 2), '-o', 'Color', [0.05 0.30 0.70], ...
'LineWidth', 2.0, 'MarkerSize', 5, 'MarkerFaceColor', [0.05 0.30 0.70], ...
'DisplayName', 'Waypoint pianificati');
disegna_punti_missione(env, goal_skipped);
axis equal; xlim(env.xlim); ylim(env.ylim);
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('Cell Decomposition e Retraction | Percorso Intelligente: %s', stato_smart_str));
legend('Location', 'northwest');
print(fig1, 'fig1_pianificazione.png', '-dpng', '-r300');

% Figura 2: Confronto traiettoria pianificata vs eseguita
fig2 = figure('Name', '2 - Traiettoria Pianificata vs Eseguita', 'Color', 'w', 'Position', [120 100 780 560]);
hold on; grid on; box on;
disegna_ostacoli(env);
if env.percorsi_intelligenti
plot(nom_waypoints(:, 1), nom_waypoints(:, 2), ':', 'Color', [0.60 0.60 0.60], ...
'LineWidth', 1.5, 'DisplayName', 'Percorso nominale');
end
plot(all_waypoints(:, 1), all_waypoints(:, 2), '--o', 'Color', [0.05 0.30 0.70], ...
'LineWidth', 1.8, 'MarkerSize', 5, 'DisplayName', 'Traiettoria pianificata');
plot(q_log(:, 1), q_log(:, 2), '-', 'Color', [0.85 0.15 0.15], ...
'LineWidth', 2.2, 'DisplayName', 'Traiettoria eseguita (differenziale)');
disegna_punti_missione(env, goal_skipped);
axis equal; xlim(env.xlim); ylim(env.ylim);
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('Traiettoria Eseguita (S_{tot} = %.2f m, T = %.2f s, v_{media} = %.2f m/s | Smart: %s)', ...
lunghezza_exec, t_log(end), vel_media_tot, stato_smart_str));
legend('Location', 'northwest');
print(fig2, 'fig2_traiettoria.png', '-dpng', '-r300');

% Figura 3: Errore di tracking, convergenza e metri percorsi istantanei/totali
fig3 = figure('Name', '3 - Errore di Tracking e Metri Percorsi', 'Color', 'w', 'Position', [160 60 780 660]);
subplot(3, 1, 1);
plot(t_log, ect_log, 'LineWidth', 1.8, 'Color', [0.05 0.30 0.70]);
grid on;
ylabel('e_{ct}(t) [m]');
title(sprintf('Errore di Tracking Cross-Track (RMSE = %.4f m, Max = %.4f m)', rmse_ct, max_ct));

subplot(3, 1, 2);
yyaxis left;
plot(t_log, dist_goal_log, 'LineWidth', 1.8);
ylabel('Distanza dal goal attivo [m]');
yyaxis right;
plot(t_log, (180 / pi) * alpha_log, 'LineWidth', 1.4, 'LineStyle', '--');
ylabel('Errore angolare \alpha [deg]');
grid on;
title('Convergenza Sequenziale alle Tappe ed Errore di Allineamento');

subplot(3, 1, 3);
hold on; grid on;
plot(t_log, s_log, 'LineWidth', 1.8, 'Color', [0.10 0.55 0.30], 'DisplayName', 'Metri percorsi istantanei s(t)');
plot([t_log(1), t_log(end)], [lunghezza_exec, lunghezza_exec], '--k', 'LineWidth', 1.3, ...
'DisplayName', sprintf('Metri percorsi totali = %.2f m', lunghezza_exec));
xlabel('Tempo t [s]');
ylabel('Distanza s(t) [m]');
title(sprintf('Distanza Percorsa nel Tempo (Totale = %.2f m in %.2f s)', lunghezza_exec, t_log(end)));
legend('Location', 'southeast');
print(fig3, 'fig3_errore_tracking.png', '-dpng', '-r300');

% Figura 4: Velocita' delle ruote, velocita' istantanea e velocita' media
fig4 = figure('Name', '4 - Velocita Ruote e Robot', 'Color', 'w', 'Position', [200 60 780 540]);
subplot(2, 1, 1);
hold on; grid on;
plot(t_log, wR_log, 'LineWidth', 1.8, 'Color', [0.05 0.35 0.75], 'DisplayName', '\omega_R (ruota destra)');
plot(t_log, wL_log, 'LineWidth', 1.8, 'Color', [0.85 0.25 0.15], 'DisplayName', '\omega_L (ruota sinistra)');
plot([t_log(1), t_log(end)], [ctrl.omega_max, ctrl.omega_max], ':k', 'LineWidth', 1.2, ...
    'DisplayName', sprintf('\pm\omega_{max} = %.1f rad/s', ctrl.omega_max));
plot([t_log(1), t_log(end)], [-ctrl.omega_max, -ctrl.omega_max], ':k', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel('\omega_R, \omega_L [rad/s]');
title('Andamento delle Velocita'' Angolari delle Ruote');
legend('Location', 'best');

subplot(2, 1, 2);
yyaxis left;
hold on; grid on;
plot(t_log, v_log, 'LineWidth', 1.8, 'DisplayName', 'Velocita'' istantanea v(t)');
plot([t_log(1), t_log(end)], [vel_media_tot, vel_media_tot], '--', 'LineWidth', 1.7, ...
'DisplayName', sprintf('Velocita'' media = %.2f m/s', vel_media_tot));
plot([t_log(1), t_log(end)], [ctrl.v_max, ctrl.v_max], ':', 'LineWidth', 1.4, ...
'DisplayName', sprintf('Crociera impostata = %.2f m/s', ctrl.v_max));
ylabel('Velocita'' lineare v [m/s]');
yyaxis right;
plot(t_log, w_log, 'LineWidth', 1.4, 'LineStyle', '-.', 'DisplayName', 'Velocita'' angolare \omega(t)');
ylabel('Velocita'' angolare \omega [rad/s]');
xlabel('Tempo t [s]');
title('Velocita'' Istantanea, Velocita'' Media e Velocita'' Angolare');
legend('Location', 'best');
print(fig4, 'fig4_velocita_ruote.png', '-dpng', '-r300');

%% 6. Animazione del moto del robot differenziale con cruscotto telemetrico e SVG custom
fig5 = figure('Name', '5 - Animazione Robot Differenziale', 'Color', 'w', 'Position', [200 60 880 700]);

% Pannello superiore: mappa 2D e animazione del robot
ax_map = axes('Parent', fig5, 'Position', [0.08 0.25 0.88 0.68]);
axes(ax_map); %#ok
hold(ax_map, 'on'); grid(ax_map, 'on'); box(ax_map, 'on');
disegna_ostacoli(env);
if env.percorsi_intelligenti
plot(nom_waypoints(:, 1), nom_waypoints(:, 2), ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 1.3, 'Parent', ax_map);
end
plot(all_waypoints(:, 1), all_waypoints(:, 2), '--', 'Color', [0.4 0.5 0.8], 'LineWidth', 1.3, 'Parent', ax_map);
disegna_punti_missione(env, goal_skipped);
axis(ax_map, 'equal'); xlim(ax_map, env.xlim); ylim(ax_map, env.ylim);
xlabel(ax_map, 'x [m]'); ylabel(ax_map, 'y [m]');

h_trace = plot(q_log(1, 1), q_log(1, 2), 'r-', 'LineWidth', 2.0, 'Parent', ax_map);

% Caricamento eventuale modello vettoriale SVG custom da config_percorso.m
svg_attivo = false;
svg_shapes = {};
if isfield(robot, 'usa_svg_custom') && robot.usa_svg_custom && ~isempty(robot.svg_path)
if exist(robot.svg_path, 'file') == 2
try
svg_shapes = carica_svg_robot(robot.svg_path, robot);
svg_attivo = ~isempty(svg_shapes);
if svg_attivo
fprintf('SVG custom caricato correttamente: %s (%d primitive).\n', ...
robot.svg_path, numel(svg_shapes));
else
warning('Nessuna primitiva SVG valida in "%s": uso grafica standard.', robot.svg_path);
end
catch ME
warning('Errore lettura SVG "%s" (%s): uso grafica standard.', robot.svg_path, ME.message);
end
else
warning('File SVG "%s" non trovato: uso grafica standard.', robot.svg_path);
end
end

if svg_attivo
h_svg_group = hgtransform('Parent', ax_map);
for s_i = 1:numel(svg_shapes)
sh = svg_shapes{s_i};
if sh.closed
patch('XData', sh.x, 'YData', sh.y, 'FaceColor', sh.face_color, ...
'EdgeColor', sh.edge_color, 'LineWidth', sh.line_width, ...
'Parent', h_svg_group);
else
line('XData', sh.x, 'YData', sh.y, 'Color', sh.edge_color, ...
'LineWidth', max(1.0, sh.line_width), 'Parent', h_svg_group);
end
end
if isfield(robot, 'mostra_cerchio_ingombro') && robot.mostra_cerchio_ingombro
h_body = plot(nan, nan, '--', 'Color', [0.15 0.40 0.70], 'LineWidth', 1.1, 'Parent', ax_map);
else
h_body = plot(nan, nan, 'LineStyle', 'none', 'Parent', ax_map);
end
h_wR   = [];
h_wL   = [];
h_head = plot(nan, nan, 'r-', 'LineWidth', 1.8, 'Parent', ax_map);
else
h_svg_group = [];
h_body = patch(nan, nan, [0.2 0.55 0.85], 'FaceAlpha', 0.45, 'EdgeColor', [0.05 0.25 0.55], 'LineWidth', 1.5, 'Parent', ax_map);
h_wR   = plot(nan, nan, 'k-', 'LineWidth', 4.5, 'Parent', ax_map);
h_wL   = plot(nan, nan, 'k-', 'LineWidth', 4.5, 'Parent', ax_map);
h_head = plot(nan, nan, 'r-', 'LineWidth', 2.2, 'Parent', ax_map);
end

% Pannello inferiore: cruscotto live per stato Percorso Intelligente, velocita' e metri
ax_hud = axes('Parent', fig5, 'Position', [0.08 0.03 0.88 0.16], 'XLim', [0 1], 'YLim', [0 1]);
axis(ax_hud, 'off');
patch([0 1 1 0], [0 0 1 1], [0.95 0.97 0.99], 'EdgeColor', [0.75 0.80 0.86], 'LineWidth', 1.2, 'Parent', ax_hud);

% Badge stato Percorso Intelligente (ATTIVO / DISATTIVO)
if env.percorsi_intelligenti
badge_col = [0.12 0.62 0.28];
badge_txt = sprintf('PERCORSO INTELLIGENTE: ATTIVO  |  Risparmio stimato: -%.2f m', metri_risparmiati);
else
badge_col = [0.82 0.42 0.10];
badge_txt = 'PERCORSO INTELLIGENTE: DISATTIVO  |  Percorso nominale su tutte le tappe';
end
patch([0.02 0.98 0.98 0.02], [0.74 0.74 0.94 0.94], badge_col, 'EdgeColor', 'none', 'Parent', ax_hud);
text(0.50, 0.84, badge_txt, 'HorizontalAlignment', 'center', 'FontSize', 9.2, ...
'FontWeight', 'bold', 'Color', 'w', 'Parent', ax_hud);

% Indicatore sinistro: Velocita' istantanea e media
text(0.02, 0.58, 'VELOCITA'' ISTANTANEA E MEDIA', 'FontSize', 8.8, 'FontWeight', 'bold', 'Color', [0.15 0.25 0.40], 'Parent', ax_hud);
patch([0.02 0.46 0.46 0.02], [0.30 0.30 0.46 0.46], [0.86 0.89 0.93], 'EdgeColor', [0.65 0.70 0.78], 'Parent', ax_hud);
h_bar_v = patch([0.02 0.02 0.02 0.02], [0.30 0.30 0.46 0.46], [0.10 0.55 0.30], 'EdgeColor', 'none', 'Parent', ax_hud);
h_txt_v = text(0.02, 0.13, 'v(t): 0.00 m/s | Media: 0.00 m/s', 'FontSize', 9.0, 'FontWeight', 'bold', 'Color', [0.05 0.15 0.30], 'Parent', ax_hud);

% Indicatore destro: Metri percorsi istantanei e totali
text(0.52, 0.58, 'METRI PERCORSI (ISTANTANEI / TOTALI)', 'FontSize', 8.8, 'FontWeight', 'bold', 'Color', [0.15 0.25 0.40], 'Parent', ax_hud);
patch([0.52 0.98 0.98 0.52], [0.30 0.30 0.46 0.46], [0.86 0.89 0.93], 'EdgeColor', [0.65 0.70 0.78], 'Parent', ax_hud);
h_bar_s = patch([0.52 0.52 0.52 0.52], [0.30 0.30 0.46 0.46], [0.05 0.35 0.75], 'EdgeColor', 'none', 'Parent', ax_hud);
h_txt_s = text(0.52, 0.13, 'Percorsi: 0.00 / 0.00 m (0%)', 'FontSize', 9.0, 'FontWeight', 'bold', 'Color', [0.05 0.15 0.30], 'Parent', ax_hud);

ang_circle = linspace(0, 2 * pi, 36);
step_anim = 4; % Disegna ogni 4 campioni (circa 12.5 Hz visivi fluidi)

for k = [1:step_anim:k_step, k_step]
if ~ishandle(h_body) || ~ishandle(h_txt_v)
break;
end
xk = q_log(k, 1);
yk = q_log(k, 2);
th = q_log(k, 3);

if svg_attivo
    rot_svg = th + robot.svg_offset_angolo_deg * pi / 180;
    set(h_svg_group, 'Matrix', makehgtform('translate', [xk, yk, 0], 'zrotate', rot_svg));
    if isfield(robot, 'mostra_cerchio_ingombro') && robot.mostra_cerchio_ingombro
        bx = xk + robot.R_body * cos(ang_circle);
        by = yk + robot.R_body * sin(ang_circle);
        set(h_body, 'XData', bx, 'YData', by);
    end
else
    bx = xk + robot.R_body * cos(ang_circle);
    by = yk + robot.R_body * sin(ang_circle);
    set(h_body, 'XData', bx, 'YData', by);

    nx_perp = -sin(th);
    ny_perp =  cos(th);
    w_len = robot.r; % Lunghezza visiva ruote proporzionale al raggio r
    cR = [xk + (robot.d / 2) * nx_perp, yk + (robot.d / 2) * ny_perp];
    cL = [xk - (robot.d / 2) * nx_perp, yk - (robot.d / 2) * ny_perp];
    set(h_wR, 'XData', [cR(1) - w_len * cos(th), cR(1) + w_len * cos(th)], ...
              'YData', [cR(2) - w_len * sin(th), cR(2) + w_len * sin(th)]);
    set(h_wL, 'XData', [cL(1) - w_len * cos(th), cL(1) + w_len * cos(th)], ...
              'YData', [cL(2) - w_len * sin(th), cL(2) + w_len * sin(th)]);
end

set(h_head, 'XData', [xk, xk + 1.2 * robot.R_body * cos(th)], ...
            'YData', [yk, yk + 1.2 * robot.R_body * sin(th)]);
set(h_trace, 'XData', q_log(1:k, 1), 'YData', q_log(1:k, 2));

% Aggiornamento cruscotto telemetrico live
v_ratio = max(0, min(1, v_log(k) / max(ctrl.v_max, 1e-6)));
x_bar_v = 0.02 + 0.44 * v_ratio;
set(h_bar_v, 'XData', [0.02 x_bar_v x_bar_v 0.02]);
set(h_txt_v, 'String', sprintf('v(t): %.2f m/s | Media: %.2f m/s (Tot: %.2f m/s)', ...
    v_log(k), vavg_log(k), vel_media_tot));

s_ratio = max(0, min(1, s_log(k) / max(lunghezza_exec, 1e-6)));
x_bar_s = 0.52 + 0.46 * s_ratio;
set(h_bar_s, 'XData', [0.52 x_bar_s x_bar_s 0.52]);
set(h_txt_s, 'String', sprintf('Percorsi: %.2f / %.2f m (%.0f%%)', ...
    s_log(k), lunghezza_exec, 100 * s_ratio));

tappa_label = env.goal_labels{leg_goal_idx(leg_log(k))};
if align_log(k)
    stato_str = sprintf('ORIENTAMENTO PRUA -> %s', tappa_label);
else
    stato_str = sprintf('NAVIGAZIONE -> %s', tappa_label);
end
title(ax_map, sprintf('%s | t = %.2f s | \\omega_R = %.2f, \\omega_L = %.2f rad/s | Smart: %s', ...
    stato_str, t_log(k), wR_log(k), wL_log(k), stato_smart_str));
drawnow;
pause(0.02);


end

if ishandle(fig5)
print(fig5, 'fig5_frame_animazione.png', '-dpng', '-r300');
end
fprintf('Figure esportate in PNG (300 dpi) nella cartella corrente.\n');

%% ========================================================================
%% FUNZIONI LOCALI DI SUPPORTO
%% ========================================================================

function [transito_ok, dpsi_deg, a_c_est] = valuta_transito_fluido(p_prev, p_curr, p_next, v_max, r_switch, ang_lim_deg, acc_lat_max)
% VALUTA_TRANSITO_FLUIDO Verifica se l'angolo tra segmento entrante e uscente
% consente di mantenere costante la velocita' senza causare slittamento.
u1 = p_curr - p_prev;
u2 = p_next - p_curr;
n1 = hypot(u1(1), u1(2));
n2 = hypot(u2(1), u2(2));
if n1 < 1e-6 || n2 < 1e-6
transito_ok = false;
dpsi_deg = 0.0;
a_c_est = 0.0;
return;
end
u1 = u1 / n1;
u2 = u2 / n2;
cos_ang = max(-1.0, min(1.0, u1(1) * u2(1) + u1(2) * u2(2)));
dpsi = acos(cos_ang);
dpsi_deg = dpsi * 180 / pi;

if dpsi_deg > ang_lim_deg
    transito_ok = false;
    a_c_est = inf;
    return;
end
if dpsi < 1e-4
    transito_ok = true;
    a_c_est = 0.0;
    return;
end

r_fillet = r_switch / tan(dpsi / 2.0);
a_c_est = (v_max^2) / max(r_fillet, 1e-4);
transito_ok = (a_c_est <= acc_lat_max);


end

function wp_out = ottimizza_smart_curve(pts, env, r_lim)
% OTTIMIZZA_SMART_CURVE Applica string-pulling (visibilita' diretta sicura) e
% corner-cutting per creare curve piu' morbide o vicine agli ostacoli.
n = size(pts, 1);
if n <= 2
wp_out = pts;
return;
end

% Passo 1: string-pulling con passo massimo 1.15 m
sp = pts(1, :);
i = 1;
while i < n
    best_j = i + 1;
    for j = n:-1:(i + 2)
        d_ij = hypot(pts(j, 1) - pts(i, 1), pts(j, 2) - pts(i, 2));
        if d_ij <= 1.15 && segmento_libero(pts(i, :), pts(j, :), env, r_lim)
            best_j = j;
            break;
        end
    end
    if hypot(pts(best_j, 1) - sp(end, 1), pts(best_j, 2) - sp(end, 2)) > 1e-3
        sp = [sp; pts(best_j, :)]; %#ok<AGROW>
    end
    i = best_j;
end

% Passo 2: arrotondamento controllato degli spigoli (corner-cutting)
if size(sp, 1) >= 3
    cc = sp(1, :);
    for k = 2:(size(sp, 1) - 1)
        A = sp(k - 1, :);
        B = sp(k, :);
        C = sp(k + 1, :);
        B1 = B + 0.25 * (A - B);
        B2 = B + 0.25 * (C - B);
        if segmento_libero(B1, B2, env, r_lim)
            cc = [cc; B1; B2]; %#ok<AGROW>
        else
            cc = [cc; B]; %#ok<AGROW>
        end
    end
    cc = [cc; sp(end, :)]; %#ok<AGROW>
else
    cc = sp;
end

% Rimozione di eventuali duplicati troppo ravvicinati
wp_out = cc(1, :);
for k = 2:size(cc, 1)
    if hypot(cc(k, 1) - wp_out(end, 1), cc(k, 2) - wp_out(end, 2)) > 0.04 || k == size(cc, 1)
        wp_out = [wp_out; cc(k, :)]; %#ok<AGROW>
    end
end


end

function ok = segmento_libero(p_a, p_b, env, r_lim)
d_ab = hypot(p_b(1) - p_a(1), p_b(2) - p_a(2));
n_s = max(12, ceil(d_ab / 0.04));
s = linspace(0, 1, n_s)';
xs = p_a(1) + s * (p_b(1) - p_a(1));
ys = p_a(2) + s * (p_b(2) - p_a(2));
ok = all(calcola_distanza_ostacoli(xs, ys, env) > r_lim);
end

function e_ct = calcola_cross_track(x, y, wp)
e_ct = inf;
for i = 1:(size(wp, 1) - 1)
x1 = wp(i, 1);     y1 = wp(i, 2);
x2 = wp(i + 1, 1); y2 = wp(i + 1, 2);
dx = x2 - x1; dy = y2 - y1;
L2 = dx^2 + dy^2;
if L2 < 1e-12
d = hypot(x - x1, y - y1);
else
s = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / L2));
d = hypot(x - (x1 + s * dx), y - (y1 + s * dy));
end
if d < e_ct
e_ct = d;
end
end
end

function disegna_ostacoli(env)
ang = linspace(0, 2 * pi, 60);
for i = 1:size(env.circles, 1)
cx = env.circles(i, 1); cy = env.circles(i, 2); R = env.circles(i, 3);
patch(cx + R * cos(ang), cy + R * sin(ang), [0.35 0.38 0.42], ...
'EdgeColor', 'k', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
for i = 1:numel(env.polygons)
poly = env.polygons{i};
patch(poly(:, 1), poly(:, 2), [0.35 0.38 0.42], ...
'EdgeColor', 'k', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
end

function disegna_punti_missione(env, goal_skipped)
plot(env.q0(1), env.q0(2), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
'DisplayName', 'HOME (partenza q_0)');
N_g = size(env.goals, 1);
if nargin < 2 || numel(goal_skipped) ~= N_g
goal_skipped = false(N_g, 1);
end

idx_int_attivi  = find(~goal_skipped(1:end-1));
idx_int_saltati = find(goal_skipped(1:end-1));

if ~isempty(idx_int_attivi)
    plot(env.goals(idx_int_attivi, 1), env.goals(idx_int_attivi, 2), 'md', ...
        'MarkerSize', 9, 'MarkerFaceColor', [0.85 0.25 0.85], ...
        'DisplayName', 'Punti intermedi visitati');
end
if ~isempty(idx_int_saltati)
    plot(env.goals(idx_int_saltati, 1), env.goals(idx_int_saltati, 2), 'kx', ...
        'MarkerSize', 10, 'LineWidth', 2.0, ...
        'DisplayName', 'Punti saltati (Percorso Intelligente)');
end
plot(env.goals(end, 1), env.goals(end, 2), 'rp', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'r', 'DisplayName', 'Traguardo finale');

for g = 1:N_g
    text(env.goals(g, 1) + 0.14, env.goals(g, 2) + 0.16, env.goal_labels{g}, ...
        'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.45 0.05 0.45]);
end


end
function shapes = carica_svg_robot(svg_path, robot)
% CARICA_SVG_ROBOT Parser vettoriale puro MATLAB per file SVG custom.
% Supporta rect, circle, ellipse, polygon, polyline e path (M,L,H,V,C,Q,Z),
% centrando e scalando automaticamente il modello su (2 * R_body * svg_scala).
txt = fileread(svg_path);
txt(txt == char(0)) = [];
if ~isempty(txt) && double(txt(1)) == 65279
    txt = txt(2:end);
end
txt = regexprep(txt, '', '');
tokens = regexp(txt, '<(rect|circle|ellipse|polygon|polyline|path)\s+([^>]*?)/?>', 'tokens', 'ignorecase');

if isempty(tokens)
    full_p = which(svg_path);
    if isempty(full_p), full_p = svg_path; end
    anteprima = regexprep(txt(1:min(100, numel(txt))), '\s+', ' ');
    fprintf('  [Debug SVG] Letti %d caratteri da "%s", 0 tag geometrici trovati.\n  [Debug SVG] Inizio file: "%s"\n', ...
        numel(txt), full_p, anteprima);
end

raw_shapes = {};
xmin = inf; xmax = -inf;
ymin = inf; ymax = -inf;

for i = 1:numel(tokens)
    tag   = lower(tokens{i}{1});
    attrs = tokens{i}{2};

    if strcmpi(parse_svg_attr(attrs, 'display'), 'none') || ...
       strcmpi(parse_svg_attr(attrs, 'visibility'), 'hidden')
        continue;
    end

    fill_str   = parse_svg_attr(attrs, 'fill');
    stroke_str = parse_svg_attr(attrs, 'stroke');
    sw_str     = parse_svg_attr(attrs, 'stroke-width');

    face_col = parse_svg_color(fill_str, [0.20 0.55 0.85]);
    if isempty(stroke_str)
        edge_col = 'none';
    else
        edge_col = parse_svg_color(stroke_str, [0.05 0.15 0.25]);
    end
    lw = str2double(regexprep(sw_str, '[^0-9\.\-+eE]', ''));
    if isnan(lw) || lw <= 0
        lw = 1.2;
    end

    polys  = {};
    closed = true;
    switch tag
        case 'rect'
            rx0 = parse_svg_num(attrs, 'x', 0);
            ry0 = parse_svg_num(attrs, 'y', 0);
            rw  = parse_svg_num(attrs, 'width', 0);
            rh  = parse_svg_num(attrs, 'height', 0);
            if rw > 0 && rh > 0
                polys = {[rx0, ry0; rx0 + rw, ry0; rx0 + rw, ry0 + rh; rx0, ry0 + rh]};
            end
        case 'circle'
            cx = parse_svg_num(attrs, 'cx', 0);
            cy = parse_svg_num(attrs, 'cy', 0);
            cr = parse_svg_num(attrs, 'r', 0);
            if cr > 0
                tt = linspace(0, 2 * pi, 36)';
                polys = {[cx + cr * cos(tt), cy + cr * sin(tt)]};
            end
        case 'ellipse'
            cx  = parse_svg_num(attrs, 'cx', 0);
            cy  = parse_svg_num(attrs, 'cy', 0);
            rxe = parse_svg_num(attrs, 'rx', 0);
            rye = parse_svg_num(attrs, 'ry', 0);
            if rxe > 0 && rye > 0
                tt = linspace(0, 2 * pi, 36)';
                polys = {[cx + rxe * cos(tt), cy + rye * sin(tt)]};
            end
        case {'polygon', 'polyline'}
            pts = parse_svg_points(parse_svg_attr(attrs, 'points'));
            if size(pts, 1) >= 2
                polys = {pts};
            end
            closed = strcmp(tag, 'polygon');
        case 'path'
            d_str = parse_svg_attr(attrs, 'd');
            [polys, has_z] = parse_svg_path_d(d_str);
            closed = has_z || ~isequal(face_col, 'none');
    end

    tr_str = parse_svg_attr(attrs, 'transform');
    if ~isempty(tr_str) && ~isempty(polys)
        m_tr = regexp(tr_str, 'translate\(\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)(?:[\s,]+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?))?\s*\)', 'tokens', 'once');
        if ~isempty(m_tr)
            tx = str2double(m_tr{1});
            ty = 0;
            if numel(m_tr) >= 2 && ~isempty(m_tr{2})
                ty = str2double(m_tr{2});
            end
            for p_i = 1:numel(polys)
                polys{p_i}(:, 1) = polys{p_i}(:, 1) + tx;
                polys{p_i}(:, 2) = polys{p_i}(:, 2) + ty;
            end
        end
    end

    for p_i = 1:numel(polys)
        pts = polys{p_i};
        if size(pts, 1) < 2
            continue;
        end
        xmin = min(xmin, min(pts(:, 1))); xmax = max(xmax, max(pts(:, 1)));
        ymin = min(ymin, min(pts(:, 2))); ymax = max(ymax, max(pts(:, 2)));
        sh.x = pts(:, 1);
        sh.y = pts(:, 2);
        sh.face_color = face_col;
        sh.edge_color = edge_col;
        sh.line_width = min(lw, 3.0);
        sh.closed = closed;
        raw_shapes{end + 1} = sh; %#ok<AGROW>
    end
end

if isempty(raw_shapes) || isinf(xmin)
    shapes = {};
    return;
end

cx_svg   = 0.5 * (xmin + xmax);
cy_svg   = 0.5 * (ymin + ymax);
span_svg = max([xmax - xmin, ymax - ymin, 1e-6]);
scala    = (2.0 * robot.R_body * robot.svg_scala) / span_svg;

shapes = cell(size(raw_shapes));
for i = 1:numel(raw_shapes)
    sh = raw_shapes{i};
    sh.x =  (sh.x - cx_svg) * scala;
    sh.y = -(sh.y - cy_svg) * scala;
    shapes{i} = sh;
end


end

function [polys, has_z] = parse_svg_path_d(d_str)
polys = {};
has_z = false;
if isempty(d_str)
return;
end

toks = regexp(d_str, '([A-Za-z])|([+-]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?)', 'match');
n_tok = numel(toks);
idx = 1;
cmd = '';
curr = [0, 0];
start_pt = [0, 0];
curr_pts = zeros(0, 2);

while idx <= n_tok
    tok = toks{idx};
    if isletter(tok(1))
        cmd = tok;
        idx = idx + 1;
        if cmd == 'Z' || cmd == 'z'
            has_z = true;
            curr = start_pt;
            if ~isempty(curr_pts)
                polys{end + 1} = curr_pts; %#ok<AGROW>
                curr_pts = zeros(0, 2);
            end
        end
        continue;
    end

    switch cmd
        case {'M', 'm'}
            if idx + 1 > n_tok, break; end
            pt = [str2double(toks{idx}), str2double(toks{idx + 1})];
            idx = idx + 2;
            if cmd == 'm', pt = curr + pt; end
            if ~isempty(curr_pts)
                polys{end + 1} = curr_pts; %#ok<AGROW>
                curr_pts = zeros(0, 2);
            end
            curr = pt;
            start_pt = pt;
            curr_pts = [curr_pts; curr]; %#ok<AGROW>
            if cmd == 'M', cmd = 'L'; else, cmd = 'l'; end

        case {'L', 'l'}
            if idx + 1 > n_tok, break; end
            pt = [str2double(toks{idx}), str2double(toks{idx + 1})];
            idx = idx + 2;
            if cmd == 'l', pt = curr + pt; end
            curr = pt;
            curr_pts = [curr_pts; curr]; %#ok<AGROW>

        case {'H', 'h'}
            val = str2double(toks{idx}); idx = idx + 1;
            if cmd == 'h', curr(1) = curr(1) + val; else, curr(1) = val; end
            curr_pts = [curr_pts; curr]; %#ok<AGROW>

        case {'V', 'v'}
            val = str2double(toks{idx}); idx = idx + 1;
            if cmd == 'v', curr(2) = curr(2) + val; else, curr(2) = val; end
            curr_pts = [curr_pts; curr]; %#ok<AGROW>

        case {'C', 'c'}
            if idx + 5 > n_tok, break; end
            p0 = curr;
            p1 = [str2double(toks{idx}),     str2double(toks{idx + 1})];
            p2 = [str2double(toks{idx + 2}), str2double(toks{idx + 3})];
            p3 = [str2double(toks{idx + 4}), str2double(toks{idx + 5})];
            idx = idx + 6;
            if cmd == 'c'
                p1 = p0 + p1; p2 = p0 + p2; p3 = p0 + p3;
            end
            tb = linspace(0, 1, 9)'; tb = tb(2:end);
            bz = ((1 - tb).^3) * p0 + (3 * ((1 - tb).^2) .* tb) * p1 + ...
                 (3 * (1 - tb) .* (tb.^2)) * p2 + (tb.^3) * p3;
            curr_pts = [curr_pts; bz]; %#ok<AGROW>
            curr = p3;

        case {'Q', 'q'}
            if idx + 3 > n_tok, break; end
            p0 = curr;
            p1 = [str2double(toks{idx}),     str2double(toks{idx + 1})];
            p2 = [str2double(toks{idx + 2}), str2double(toks{idx + 3})];
            idx = idx + 4;
            if cmd == 'q'
                p1 = p0 + p1; p2 = p0 + p2;
            end
            tb = linspace(0, 1, 9)'; tb = tb(2:end);
            bz = ((1 - tb).^2) * p0 + (2 * (1 - tb) .* tb) * p1 + (tb.^2) * p2;
            curr_pts = [curr_pts; bz]; %#ok<AGROW>
            curr = p2;

        case {'A', 'a'}
            if idx + 6 > n_tok, break; end
            pt = [str2double(toks{idx + 5}), str2double(toks{idx + 6})];
            idx = idx + 7;
            if cmd == 'a', pt = curr + pt; end
            curr = pt;
            curr_pts = [curr_pts; curr]; %#ok<AGROW>

        otherwise
            idx = idx + 1;
    end
end

if ~isempty(curr_pts)
    polys{end + 1} = curr_pts; %#ok<AGROW>
end


end

function val = parse_svg_attr(attrs, name)
val = '';
pat = ['(?:^|[\s/])' name '\s*=\s*"([^"])"'];
m = regexp(attrs, pat, 'tokens', 'once', 'ignorecase');
if isempty(m)
    pat_sq = ['(?:^|[\s/])' name '\s=\s*''([^''])'''];
    m = regexp(attrs, pat_sq, 'tokens', 'once', 'ignorecase');
end
if ~isempty(m)
    val = strtrim(m{1});
    return;
end
m_st = regexp(attrs, '(?:^|\s)style\s=\s*"([^"])"', 'tokens', 'once', 'ignorecase');
if ~isempty(m_st)
    m_sub = regexp(m_st{1}, ['(?:^|;)\s' name '\s*:\s*([^;]+)'], 'tokens', 'once', 'ignorecase');
    if ~isempty(m_sub)
        val = strtrim(m_sub{1});
    end
end
end

function num = parse_svg_num(attrs, name, default_val)
s = parse_svg_attr(attrs, name);
if isempty(s)
num = default_val;
return;
end
s_clean = regexprep(s, '[^0-9.-+eE]', '');
num = str2double(s_clean);
if isnan(num)
num = default_val;
end
end

function pts = parse_svg_points(pts_str)
if isempty(pts_str)
pts = zeros(0, 2);
return;
end
nums = str2double(regexp(pts_str, '[+-]?(?:\d+.\d*|.\d+|\d+)(?:[eE][+-]?\d+)?', 'match'));
n_pairs = floor(numel(nums) / 2);
if n_pairs < 1
pts = zeros(0, 2);
return;
end
pts = reshape(nums(1:(2 * n_pairs)), 2, n_pairs)';
end

function col = parse_svg_color(col_str, default_col)
if isempty(col_str)
col = default_col;
return;
end
s = lower(strtrim(col_str));
if strcmp(s, 'none') || strcmp(s, 'transparent')
col = 'none';
return;
end
if s(1) == '#'
if numel(s) == 7
col = [hex2dec(s(2:3)), hex2dec(s(4:5)), hex2dec(s(6:7))] / 255;
return;
elseif numel(s) == 4
col = [hex2dec([s(2) s(2)]), hex2dec([s(3) s(3)]), hex2dec([s(4) s(4)])] / 255;
return;
end
end
switch s
case 'black',     col = [0.00 0.00 0.00];
case 'white',     col = [1.00 1.00 1.00];
case 'red',       col = [0.85 0.15 0.15];
case 'green',     col = [0.15 0.65 0.25];
case 'blue',      col = [0.15 0.45 0.85];
case 'yellow',    col = [0.95 0.80 0.10];
case 'orange',    col = [0.95 0.50 0.10];
case 'cyan',      col = [0.10 0.75 0.85];
case 'magenta',   col = [0.80 0.20 0.75];
case {'gray', 'grey'}, col = [0.50 0.50 0.50];
case {'darkgray', 'darkgrey'}, col = [0.25 0.28 0.32];
case {'lightgray', 'lightgrey', 'silver'}, col = [0.80 0.82 0.85];
case 'navy',      col = [0.05 0.15 0.40];
otherwise,        col = default_col;
end
end
