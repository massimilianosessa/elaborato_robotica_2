function [waypoints, grid_data] = pianifica_celle_retraction(env, robot, p_start, p_goal, r_safe_override, lambda_clr_override)
% PIANIFICA_CELLE_RETRACTION Implementa:
% 1. Decomposizione in celle regolari (libere/occupate nel C-space);
% 2. Costruzione del grafo di adiacenza 8-connesso;
% 3. Mappa di retraction tramite scheletro mediale di clearance;
% 4. Ricerca cammino ottimo con Dijkstra tra p_start e p_goal e decimazione waypoint.

if nargin < 3 || isempty(p_start)
    p_start = env.q0(1:2)';
else
    p_start = p_start(:)';
end
if nargin < 4 || isempty(p_goal)
    p_goal = env.qf(1:2)';
else
    p_goal = p_goal(:)';
end
if nargin < 5 || isempty(r_safe_override)
    r_safe_val = robot.R_safe;
else
    r_safe_val = r_safe_override;
end
if nargin < 6 || isempty(lambda_clr_override)
    lambda_clr = 0.10;
else
    lambda_clr = lambda_clr_override;
end

%% 1. Griglia delle celle libere e occupate nello spazio C_free
res = 0.20; % Dimensione della cella elementare [m]
xc = (env.xlim(1) + res / 2) : res : (env.xlim(2) - res / 2);
yc = (env.ylim(1) + res / 2) : res : (env.ylim(2) - res / 2);
[X, Y] = meshgrid(xc, yc);
[ny, nx] = size(X);
N = ny * nx;

D_obs     = calcola_distanza_ostacoli(X, Y, env);
free_mask = D_obs > r_safe_val;
clearance = max(0, D_obs - robot.R_body);

% Associazione di partenza e obiettivo alle rispettive celle libere
[~, c0] = min(abs(xc - p_start(1)));
[~, r0] = min(abs(yc - p_start(2)));
[~, cf] = min(abs(xc - p_goal(1)));
[~, rf] = min(abs(yc - p_goal(2)));

if ~free_mask(r0, c0)
    [r0, c0] = trova_cella_libera_vicina(p_start, X, Y, free_mask, 'partenza');
end
if ~free_mask(rf, cf)
    [rf, cf] = trova_cella_libera_vicina(p_goal, X, Y, free_mask, 'obiettivo');
end

%% 2. Mappa di Retraction (scheletro mediale di clearance)
skel_mask = false(ny, nx);
for r = 2:(ny - 1)
    for c = 2:(nx - 1)
        if ~free_mask(r, c)
            continue;
        end
        clr_c = clearance(r, c);
        ridge_h  = (clr_c >= clearance(r, c - 1)) && (clr_c >= clearance(r, c + 1)) && ...
                   ((clr_c > clearance(r, c - 1)) || (clr_c > clearance(r, c + 1)));
        ridge_v  = (clr_c >= clearance(r - 1, c)) && (clr_c >= clearance(r + 1, c)) && ...
                   ((clr_c > clearance(r - 1, c)) || (clr_c > clearance(r + 1, c)));
        ridge_d1 = (clr_c >= clearance(r - 1, c - 1)) && (clr_c >= clearance(r + 1, c + 1)) && ...
                   ((clr_c > clearance(r - 1, c - 1)) || (clr_c > clearance(r + 1, c + 1)));
        ridge_d2 = (clr_c >= clearance(r - 1, c + 1)) && (clr_c >= clearance(r + 1, c - 1)) && ...
                   ((clr_c > clearance(r - 1, c + 1)) || (clr_c > clearance(r + 1, c - 1)));
        if ridge_h || ridge_v || ridge_d1 || ridge_d2
            skel_mask(r, c) = true;
        end
    end
end

%% 3. Grafo di adiacenza 8-connesso e Algoritmo di Dijkstra pesato sulla Retraction
dirs = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];

start_id = sub2ind([ny, nx], r0, c0);
goal_id  = sub2ind([ny, nx], rf, cf);

dist_g  = inf(N, 1);
prev_id = zeros(N, 1);
visited = false(N, 1);
dist_g(start_id) = 0.0;

for iter = 1:N %#ok<NASGU>
    tmp = dist_g;
    tmp(visited) = inf;
    [min_d, u] = min(tmp);
    if isinf(min_d) || u == goal_id
        break;
    end
    visited(u) = true;

    [ru, cu] = ind2sub([ny, nx], u);
    for d_i = 1:8
        dr = dirs(d_i, 1);
        dc = dirs(d_i, 2);
        rv = ru + dr;
        cv = cu + dc;
        if rv < 1 || rv > ny || cv < 1 || cv > nx
            continue;
        end
        if ~free_mask(rv, cv)
            continue;
        end
        % Impedisce il taglio diagonale di spigoli occupati
        if dr ~= 0 && dc ~= 0
            if ~free_mask(ru + dr, cu) || ~free_mask(ru, cu + dc)
                continue;
            end
        end
        v = sub2ind([ny, nx], rv, cv);
        if visited(v)
            continue;
        end

        step_len = hypot(X(rv, cv) - X(ru, cu), Y(rv, cv) - Y(ru, cu));
        clr_v = max(clearance(rv, cv), 0.05);
        % Costo composto: distanza euclidea + attrazione verso l'asse mediale di retraction
        w_uv = step_len * (1.0 + lambda_clr / (clr_v^2));

        alt = min_d + w_uv;
        if alt < dist_g(v)
            dist_g(v)  = alt;
            prev_id(v) = u;
        end
    end
end

if isinf(dist_g(goal_id))
    error('Nessun percorso collision-free trovato tra [%.2f, %.2f] e [%.2f, %.2f].', ...
        p_start(1), p_start(2), p_goal(1), p_goal(2));
end

% Ricostruzione della sequenza di celle da start a goal
path_ids = [];
curr = goal_id;
while curr ~= 0
    path_ids = [curr; path_ids]; %#ok<AGROW>
    if curr == start_id
        break;
    end
    curr = prev_id(curr);
end

raw_path = [p_start; [X(path_ids), Y(path_ids)]; p_goal];

%% 4. Semplificazione dei waypoint lungo lo scheletro retratto
waypoints = semplifica_waypoint(raw_path, env, r_safe_val);

% Esportazione dati griglia per visualizzazione
grid_data.xc        = xc;
grid_data.yc        = yc;
grid_data.X         = X;
grid_data.Y         = Y;
grid_data.free_mask = free_mask;
grid_data.skel_mask = skel_mask;
grid_data.D_obs     = D_obs;
grid_data.clearance = clearance;
grid_data.raw_path  = raw_path;


end

%% Funzioni locali di supporto
function [r_best, c_best] = trova_cella_libera_vicina(pt, X, Y, free_mask, etichetta)
if ~any(free_mask(:))
error('Lo spazio delle configurazioni libero C_free e'' vuoto.');
end
[fr, fc] = find(free_mask);
dists = hypot(X(free_mask) - pt(1), Y(free_mask) - pt(2));
[d_min, idx] = min(dists);
if d_min > 0.45
error('Il punto di %s [%.2f, %.2f] ricade all''interno di un ostacolo o margine di sicurezza (dist = %.2f m).', ...
etichetta, pt(1), pt(2), d_min);
end
r_best = fr(idx);
c_best = fc(idx);
end

function wp = semplifica_waypoint(path_xy, env, r_lim)
% Riduce i gradini della griglia preservando la sicurezza, la vicinanza
% all'asse mediale retratto (deviazione <= 0.30 m) e una distanza massima
% tra waypoint consecutivi (<= 0.90 m) per contenere il raggio di curvatura.
n_pts = size(path_xy, 1);
L_seg_max = 0.90;
wp = path_xy(1, :);
i = 1;
while i < n_pts
best_j = i + 1;
for j = n_pts:-1:(i + 2)
p_a = path_xy(i, :);
p_b = path_xy(j, :);
if hypot(p_b(1) - p_a(1), p_b(2) - p_a(2)) <= L_seg_max && ...
segmento_sicuro(p_a, p_b, env, r_lim) && ...
deviazione_max(p_a, p_b, path_xy(i:j, :)) <= 0.30
best_j = j;
break;
end
end
wp = [wp; path_xy(best_j, :)]; %#ok
i = best_j;
end
end

function ok = segmento_sicuro(p_a, p_b, env, r_lim)
d_ab = hypot(p_b(1) - p_a(1), p_b(2) - p_a(2));
n_s = max(10, ceil(d_ab / 0.05));
s = linspace(0, 1, n_s)';
xs = p_a(1) + s * (p_b(1) - p_a(1));
ys = p_a(2) + s * (p_b(2) - p_a(2));
ok = all(calcola_distanza_ostacoli(xs, ys, env) > r_lim);
end

function d_max = deviazione_max(p_a, p_b, pts)
dx = p_b(1) - p_a(1);
dy = p_b(2) - p_a(2);
L = hypot(dx, dy);
if L < 1e-9
d_max = 0.0;
return;
end
d_max = max(abs(dy * (pts(:, 1) - p_a(1)) - dx * (pts(:, 2) - p_a(2))) / L);
end
