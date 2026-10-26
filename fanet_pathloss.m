function PL = fanet_pathloss(P, A, B)
%FANET_PATHLOSS  Deterministic path loss, Eq. (4.1) without shadowing.
%   A, B : M x 3 endpoint positions.  Returns M x 1 path loss in dB:
%   log-distance term plus the excess loss A_ij of every obstacle whose
%   3-D box is crossed by the straight segment A-B (exact slab test).
d  = sqrt(sum((A - B).^2, 2));
d  = max(d, 1);
PL = P.PL0 + 10 * P.eta * log10(d / P.d0);
D  = B - A;
for o = 1:size(P.obst, 1)
    ob = P.obst(o, :);
    lo = [ob(1) ob(3) 0];
    hi = [ob(2) ob(4) ob(5)];
    tmin = zeros(size(d));
    tmax = ones(size(d));
    for c = 1:3
        dc  = D(:, c);
        ac  = A(:, c);
        par = abs(dc) < 1e-9;
        dc(par) = 1;
        t1 = (lo(c) - ac) ./ dc;
        t2 = (hi(c) - ac) ./ dc;
        tl = min(t1, t2);
        th = max(t1, t2);
        tl(par) = -Inf;
        th(par) =  Inf;
        outside = par & (ac < lo(c) | ac > hi(c));
        tmin = max(tmin, tl);
        tmax = min(tmax, th);
        tmax(outside) = -1;
    end
    hit = tmin <= tmax;
    PL  = PL + ob(6) * hit;
end
end
