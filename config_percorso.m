function cfg = config_percorso()
% CONFIG_PERCORSO File di configurazione centralizzato del percorso, delle
% dimensioni fisiche del robot e della grafica SVG custom.
%
% Struttura del percorso:
%   - Il PRIMO punto della missione e' sempre 'cfg.home' (posa di partenza).
%   - L'ULTIMO elemento di 'cfg.percorso' rappresenta il TRAGUARDO finale.
%   - Gli elementi intermedi possono essere coordinate [x, y] oppure la
%     stringa 'home' per comandare al robot il ritorno alla base 'home'.
%
% Esempi d'uso per 'cfg.percorso':
%   1) Percorso con punti intermedi e traguardo finale:
%      cfg.percorso = { [1.1, 4.1], [4.0, 4.1], [6.4, 4.3], [6.6, 7.2], [9.2, 1.2] };
%   2) Percorso con ritorno intermedio a home e poi traguardo:
%      cfg.percorso = { [1.1, 4.1], [4.0, 4.1], 'home', [9.2, 1.2] };
%   3) Percorso con ritorno finale a home (traguardo = home):
%      cfg.percorso = { [1.1, 4.1], [4.0, 4.1], [9.2, 1.2], 'home' };

%% 1. Punto di partenza (HOME) [x; y; theta]
cfg.home = [1.0; 1.2; 0.0];

%% 2. Sequenza del percorso (punti intermedi + ultimo punto = traguardo)
% Inserire 'home' in qualsiasi posizione per far tornare il robot a home.
cfg.percorso = {
    [1.1, 4.1], ...   % Punto intermedio 1 (curva attorno a O1)
    [4.0, 4.1], ...   % Punto intermedio 2 (corridoio centrale)
    [6.4, 4.3], ...   % Punto intermedio 3 (uscita corridoio centrale)
    [6.6, 7.2], ...   % Punto intermedio 4 (deviazione lunga attorno a O2)
    [9.2, 6.2],'home' % Traguardo finale (o inserire 'home' per tornare alla base
    };

%% 3. Funzionalita' PERCORSI INTELLIGENTI (true = ATTIVO, false = DISATTIVO)
% Se attivo: qualora un punto intermedio allunghi il percorso verso il
% punto successivo, il robot puo' decidere di non passarci sopra e
% tracciare una curva piu' morbida o piu' vicina agli ostacoli per
% mantenere velocita' costante e ridurre i metri percorsi totali.
cfg.percorsi_intelligenti = false;

% Margine minimo dagli ostacoli ammesso in modalita' intelligente [m]
% (il margine nominale standard e' 0.14 m; qui 0.06 m oltre R_body = 0.24 m)
cfg.margine_intelligente = 0.06;

%% 4. Parametri anti-slittamento ruote nei punti intermedi e velocita'
% Se il cambio di direzione al punto intermedio e' entro queste soglie,
% il robot NON decelera e prosegue a velocita' costante.
cfg.angolo_max_no_slittamento_deg = 45.0; % Deflessione max senza decelerare [deg]
cfg.a_lat_max                     = 1.25; % Accelerazione centripeta max anti-slittamento [m/s^2]

% Fattore scalare di velocita' (1.0 = nominale, >1 piu' veloce, <1 piu' lento)
cfg.fattore_velocita = 2.0;

%% 5. Dimensioni geometriche e fisiche del robot differenziale
cfg.robot.r         = 0.08;   % Raggio delle ruote motrici [m]
cfg.robot.d         = 0.36;   % Asse / carreggiata tra le due ruote [m]
cfg.robot.R_body    = 0.24;   % Raggio d'ingombro circolare del robot [m]
cfg.robot.margin    = 0.14;   % Margine di sicurezza nominale dagli ostacoli [m]
cfg.robot.omega_max = 12.0;   % Velocita' angolare massima motori ruote [rad/s]

%% 6. Parametrizzazione SVG custom per la visualizzazione del robot
% Impostare 'usa_svg_custom = true' e indicare il percorso del file SVG.
% Lo sprite viene scalato automaticamente su (2 * R_body * svg_scala).
cfg.robot.usa_svg_custom          = true;
cfg.robot.svg_path                = 'robot_custom.svg';
cfg.robot.svg_scala               = 1.0;   % Fattore moltiplicativo rispetto a 2*R_body
cfg.robot.svg_offset_angolo_deg   = 0.0;   % Correzione orientamento SVG [deg] (0 = prua verso +X)
cfg.robot.mostra_cerchio_ingombro = true;  % Disegna cerchio tratteggiato di collisione R_body
end
