function D_obs = calcola_distanza_ostacoli(X, Y, env)
% CALCOLA_DISTANZA_OSTACOLI Calcola la distanza euclidea con segno da pareti,
% ostacoli circolari e ostacoli poligonali in forma vettorizzata.

% Distanza dai bordi rettangolari dell'ambiente
d_wall_x = min(X - env.xlim(1), env.xlim(2) - X);
d_wall_y = min(Y - env.ylim(1), env.ylim(2) - Y);
D_obs = min(d_wall_x, d_wall_y);

% Distanza dagli ostacoli circolari
for i = 1:size(env.circles, 1)
    cx = env.circles(i, 1);
    cy = env.circles(i, 2);
    R  = env.circles(i, 3);
    d_c = hypot(X - cx, Y - cy) - R;
    D_obs = min(D_obs, d_c);
end

% Distanza dagli ostacoli poligonali
for i = 1:numel(env.polygons)
    poly = env.polygons{i};
    nv = size(poly, 1);
    d_poly = inf(size(X));
    for e = 1:nv
        e_next = mod(e, nv) + 1;
        x1 = poly(e, 1);      y1 = poly(e, 2);
        x2 = poly(e_next, 1); y2 = poly(e_next, 2);
        dx = x2 - x1; dy = y2 - y1;
        L2 = dx^2 + dy^2;
        s = max(0, min(1, ((X - x1).*dx + (Y - y1).*dy) ./ L2));
        d_seg = hypot(X - (x1 + s.*dx), Y - (y1 + s.*dy));
        d_poly = min(d_poly, d_seg);
    end
    in_p = inpolygon(X, Y, poly(:,1), poly(:,2));
    d_poly(in_p) = -d_poly(in_p);
    D_obs = min(D_obs, d_poly);
end


end
