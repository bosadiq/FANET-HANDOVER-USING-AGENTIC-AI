function [tgt, cnt, ctgt] = fanet_decide(P, mode, theta, S, i)
%FANET_DECIDE  One handover decision of the network agent on UAV i.
%   mode  : 'rssi' | 'a3' | 'agent' | 'agent_nopred' | 'agent_noads' | 'agent_nogate'
%   theta : policy parameters [w_gamma w_life w_load w_energy w_hop w_stay w_video margin]
%   S     : struct with the local observation o_i(t) and neighbour advertisements
%   tgt   : 0 = stay (a_stay), otherwise target node j (a_j)
%   cnt,ctgt : updated A3 time-to-trigger state of UAV i
N = P.N; G = P.G;
cur  = S.serv(i);
g    = S.gam(i, :);                       % SINR to every node
tgt  = 0;  cnt = S.a3cnt(i);  ctgt = S.a3tgt(i);

usePred = ~strcmp(mode, 'agent_nopred');
useAds  = ~strcmp(mode, 'agent_noads');
useGate = usePred && ~strcmp(mode, 'agent_nogate');
isAgent = strncmp(mode, 'agent', 5);

curOK = (cur == G || S.alive(cur)) && g(cur) >= P.Gmin;

% ---- candidate set C_i(t): usable, connected, loop-free ------------------
ok = false(1, N + 1);
for j = 1:N + 1
    if j == i || j == cur, continue; end
    if j <= N && ~S.alive(j), continue; end
    if g(j) < P.Gmin, continue; end
    if j <= N && ~S.valid(j), continue; end          % j has no path to the GCS
    if any(fanet_chain(S.serv, j, G, P.Hmax + 1) == i), continue; end
    if isAgent && useAds && j <= N                    % constraint C3
        if S.lam(j) > P.lamMax || S.E(j) < P.Emin, continue; end
    end
    ok(j) = true;
end
% keep the k strongest UAV neighbours (+ the GCS if usable)
uav = find(ok(1:N));
if numel(uav) > P.k
    [~, ord] = sort(g(uav), 'descend');
    uav = uav(ord(1:P.k));
end
cand = uav;
if ok(G), cand = [cand G]; end
if isempty(cand), cnt = 0; return; end

% ------------------------------------------------------------------------
if strcmp(mode, 'rssi')
    [gb, k] = max(g(cand));  best = cand(k);
    if ~curOK || (g(cur) < P.rssiThr && gb > g(cur))
        tgt = best;
    end
    return;
end

doA3 = strcmp(mode, 'a3');
if isAgent && useGate && curOK && S.kap(i, cur) < P.kapMin
    doA3 = true;                                       % confidence gate, Eq. (4.17)
end
if doA3
    [gb, k] = max(g(cand));  best = cand(k);
    if ~curOK
        tgt = best;  cnt = 0;  ctgt = 0;
    elseif gb > g(cur) + P.a3Hys
        if best == ctgt, cnt = cnt + 1; else cnt = 1; ctgt = best; end
        if cnt >= P.a3TTT, tgt = best; cnt = 0; ctgt = 0; end
    else
        cnt = 0; ctgt = 0;
    end
    return;
end

% ---- learned / utility policy pi_theta ---------------------------------
u = -Inf(1, N + 1);
for j = [cand cur]
    if j == cur && ~curOK, continue; end
    f1 = min(max((g(j) - P.Gmin) / 20, 0), 1);
    if usePred
        f2 = min(S.lh(i, j), P.Lmax) / P.Lmax;
    else
        f2 = f1;
    end
    if useAds
        if j == G, lam = 0; E = 1; hop = 0;
        else lam = S.lam(j); E = S.E(j); hop = S.hop(j); end
    else                                   % no load / energy advertisements
        lam = 0; E = 1;
        if j == G, hop = 0; else hop = S.hop(j); end
    end
    f = [f1, f2, 1 - lam, E, -(hop + 1) / P.Hmax, double(j == cur), S.tau(i) * f2];
    u(j) = f * theta(1:7)';
end
[ub, k] = max(u(cand));  best = cand(k);
if ~curOK
    tgt = best;                                        % forced re-selection
elseif S.dwell(i) >= P.minDwell && ub > u(cur) + abs(theta(8))
    tgt = best;
end
end
