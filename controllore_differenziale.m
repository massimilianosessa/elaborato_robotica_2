function [wR, wL, v_cmd, w_cmd, rho, alpha] = controllore_differenziale(q, target_xy, params)
% CONTROLLORE_DIFFERENZIALE Legge di controllo polare:
%   v = k_rho * rho  (oppure v = v_max in attraversamento fluido senza decelerazione)
%   omega = k_alpha * alpha
% Supporta:
%   - 'solo_orientamento': rotazione pura sul posto (v_cmd = 0, wR = -wL)
%   - 'mantieni_velocita': attraversamento punti intermedi senza decelerare
%     quando la variazione di traiettoria non provoca slittamento ruote.

if ~isfield(params, 'v_max')
    params.v_max = 0.75;
end
if ~isfield(params, 'solo_orientamento')
    params.solo_orientamento = false;
end
if ~isfield(params, 'mantieni_velocita')
    params.mantieni_velocita = false;
end

dx = target_xy(1) - q(1);
dy = target_xy(2) - q(2);

% Coordinate polari locali rispetto al waypoint target
rho   = hypot(dx, dy);
phi   = atan2(dy, dx);
alpha = atan2(sin(phi - q(3)), cos(phi - q(3)));

% Legge di controllo cinematica (modello differenziale)
if params.solo_orientamento || abs(alpha) >= pi / 2
    % Rotazione pura sul posto (wR = -wL, v = 0) per orientare la prua
    v_cmd = 0.0;
elseif params.mantieni_velocita
    % Passaggio fluido su punto/waypoint intermedio senza decelerazione
    v_cmd = params.v_max;
else
    % Avvicinamento con decelerazione progressiva fino al traguardo o svolta stretta
    v_cmd = min(params.k_rho * rho, params.v_max);
end
w_cmd = params.k_alpha * alpha;

% Conversione da (v, omega) alle velocita angolari delle due ruote
wR_raw = (v_cmd + (params.d / 2) * w_cmd) / params.r;
wL_raw = (v_cmd - (params.d / 2) * w_cmd) / params.r;

% Saturazione proporzionale: preserva la curvatura omega/v se un motore satura
max_w = max(abs(wR_raw), abs(wL_raw));
if max_w > params.omega_max
    scala = params.omega_max / max_w;
    wR = scala * wR_raw;
    wL = scala * wL_raw;
else
    wR = wR_raw;
    wL = wL_raw;
end



end
