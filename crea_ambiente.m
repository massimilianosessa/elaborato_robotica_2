function [env, robot] = crea_ambiente()
% CREA_AMBIENTE Definisce workspace, ostacoli circolari/poligonali,
% carica la configurazione percorso da config_percorso.m (home, punti
% intermedi, traguardo, percorsi intelligenti) e imposta i parametri fisici.

% Caricamento file di configurazione esterno del percorso
cfg = config_percorso();

% Dimensioni piano rettangolare di lavoro [m]
env.xlim = [0.0, 10.0];
env.ylim = [0.0, 8.0];

% Primo punto = HOME (configurazione iniziale q0 = [x0; y0; theta0])
env.home = cfg.home(:);
env.q0   = env.home;

% Parsing della sequenza punti: supporta coordinate [x, y] o stringa 'home'
if ~isfield(cfg, 'percorso') || isempty(cfg.percorso)
    error('Configurare almeno un punto traguardo in config_percorso.m -> cfg.percorso.');
end

N_g = numel(cfg.percorso);
env.goals        = zeros(N_g, 2);
env.is_home_goal = false(N_g, 1);
env.goal_labels  = cell(N_g, 1);

for g = 1:N_g
    elem = cfg.percorso{g};
    if (ischar(elem) || isstring(elem)) && strcmpi(strtrim(char(elem)), 'home')
        env.goals(g, :)     = env.home(1:2)';
        env.is_home_goal(g) = true;
        if g == N_g
            env.goal_labels{g} = 'HOME (Traguardo)';
        else
            env.goal_labels{g} = 'HOME';
        end
    elseif isnumeric(elem) && numel(elem) == 2
        env.goals(g, :) = reshape(elem, 1, 2);
        if g == N_g
            env.goal_labels{g} = 'TRAGUARDO';
        else
            env.goal_labels{g} = sprintf('P%d', g);
        end
    else
        error('Elemento %d non valido in cfg.percorso: usare [x, y] o ''home''.', g);
    end
end

% Configurazione finale complessiva (traguardo)
env.qf = [env.goals(end, :)'; 0.0];

% Parametri di configurazione navigazione intelligente e anti-slittamento
env.percorsi_intelligenti         = cfg.percorsi_intelligenti;
env.margine_intelligente          = cfg.margine_intelligente;
env.angolo_max_no_slittamento_deg = cfg.angolo_max_no_slittamento_deg;
env.a_lat_max                     = cfg.a_lat_max;
env.fattore_velocita              = cfg.fattore_velocita;

% Ostacoli circolari: ogni riga e [cx, cy, raggio]
env.circles = [
    2.5, 2.2, 0.75;
    7.5, 5.6, 0.85;
    5.0, 6.3, 0.70
    ];

% Ostacoli poligonali: cell array di matrici [x_i, y_i] (vertici in ordine)
env.polygons = {
    [4.4, 1.5; 5.8, 1.5; 5.8, 3.6; 4.4, 3.6];
    [2.2, 4.6; 3.5, 4.8; 3.8, 6.2; 2.6, 6.5; 1.9, 5.5];
    [7.0, 1.5; 8.5, 1.5; 8.5, 2.8; 7.3, 3.0]
    };

% Parametri fisici e grafici del robot letti da config_percorso.m
robot.r            = cfg.robot.r;
robot.d            = cfg.robot.d;
robot.R_body       = cfg.robot.R_body;
robot.margin       = cfg.robot.margin;
robot.R_safe       = robot.R_body + robot.margin;             % Raggio gonfiaggio C-space nominale [m]
robot.R_safe_smart = robot.R_body + cfg.margine_intelligente; % Raggio ridotto modalita' intelligente [m]
robot.omega_max    = cfg.robot.omega_max;

% Configurazione SVG custom per il rendering del robot
robot.usa_svg_custom          = cfg.robot.usa_svg_custom;
robot.svg_path                = cfg.robot.svg_path;
robot.svg_scala               = cfg.robot.svg_scala;
robot.svg_offset_angolo_deg   = cfg.robot.svg_offset_angolo_deg;
robot.mostra_cerchio_ingombro = cfg.robot.mostra_cerchio_ingombro;



end
