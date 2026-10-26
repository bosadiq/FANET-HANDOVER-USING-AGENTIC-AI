function R = fanet_run(P, ENV, mode, theta)
%FANET_RUN  Simulate one mission with every UAV running the same policy.
%   Implements the Dec-POMDP of Section 4: each slot every UAV observes
%   locally (SINR, predicted lifetime, neighbour advertisements), picks
%   a_stay or a_j, handovers complete after P.Dho slots (make-before-break),
%   and the team reward of Eq. (4.11)-(4.15) is accumulated.
N = P.N; G = P.G; T = P.T; Ts = P.Ts; w = P.w;
if nargin < 4 || isempty(theta), theta = P.theta0; end

E     = ENV.E0;
alive = true(N, 1);
serv  = init_association(P, ENV.GAM(:, :, 1));
prevServ = zeros(N, 1);  tlast = -1e9 * ones(N, 1);
pendT = zeros(N, 1);  pendTime = zeros(N, 1);
a3cnt = zeros(N, 1);  a3tgt = zeros(N, 1);
valid = false(N, 1);  flowPrev = zeros(N, 1);

% accumulators
Jtot = 0; sumU = 0; sumDel = 0; sumGen = 0; sumRho = 0; sumSig = 0;
nHOok = 0; nHOatt = 0; nHOfail = 0; vidDel = 0; vidGen = 0; vidSlots = 0;
lvSum = 0; lvCnt = 0; validCnt = 0; aliveCnt = 0; nDeaths = 0;
SERV = zeros(N, T); GS = NaN(N, T); HOLOG = zeros(0, 4);

for t = 1:T
    Gam = ENV.GAM(:, :, t);
    % ---- alive / failure ------------------------------------------------
    if strcmp(ENV.scen, 'fail') && t >= P.failSlot && alive(P.failUAV)
        alive(P.failUAV) = false;
    end
    tau = zeros(N, 1);
    if t >= P.videoSlots(1) && t <= P.videoSlots(2), tau(P.videoUAV) = 1; end

    % ---- apply handovers whose execution delay has elapsed -------------
    rho = zeros(N, 1);  hoNow = false(N, 1);
    for i = 1:N
        if pendT(i) > 0 && t >= pendTime(i)
            tg = pendT(i);  pendT(i) = 0;
            okT = alive(i) && (tg == G || alive(tg)) && Gam(i, tg) >= P.Gmin ...
                  && ~any(fanet_chain(serv, tg, G, P.Hmax + 1) == i);
            if okT
                rho(i) = double(tg == prevServ(i) && (t - tlast(i)) < P.Tpp);
                prevServ(i) = serv(i);  serv(i) = tg;  tlast(i) = t;
                hoNow(i) = true;  nHOok = nHOok + 1;
                HOLOG(end + 1, :) = [t i prevServ(i) tg]; %#ok<AGROW>
            else
                nHOfail = nHOfail + 1;
            end
        end
    end

    % ---- advertisements (state at start of slot) ------------------------
    hop = Inf(1, N + 1);  hop(G) = 0;
    for j = 1:N
        if valid(j), hop(j) = numel(fanet_chain(serv, j, G, P.Hmax + 1)) - 1; end
    end
    S.gam = Gam; S.lh = ENV.LH(:, :, t); S.kap = ENV.KAP(:, :, t);
    S.serv = serv; S.alive = alive; S.valid = valid;
    S.lam = min(flowPrev / P.nodeCap, 1); S.E = E; S.hop = hop;
    S.tau = tau; S.dwell = t - tlast; S.a3cnt = a3cnt; S.a3tgt = a3tgt;

    % ---- decisions ------------------------------------------------------
    nSig = zeros(N, 1);  nAtt = zeros(N, 1);
    for i = 1:N
        if ~alive(i) || pendT(i) > 0, continue; end
        [tg, c, ct] = fanet_decide(P, mode, theta, S, i);
        a3cnt(i) = c;  a3tgt(i) = ct;
        if tg > 0 && tg ~= serv(i)
            pendT(i) = tg;  pendTime(i) = t + P.Dho;
            nSig(i) = P.msgsPerHO;  nAtt(i) = 1;  nHOatt = nHOatt + 1;
        end
    end

    % ---- evaluate delivery with the current serving chains --------------
    valid = false(N, 1);  paths = cell(N, 1);
    for o = 1:N
        if ~alive(o), continue; end
        cur = o;  nodes = zeros(1, 0);  reached = false;
        for h = 1:P.Hmax
            nx = serv(cur);
            if nx == 0, break; end
            if nx <= N && ~alive(nx), break; end
            if Gam(cur, nx) < P.Gmin, break; end
            nodes(end + 1) = cur; %#ok<AGROW>
            if nx == G, reached = true; break; end
            cur = nx;
        end
        valid(o) = reached;
        if reached, paths{o} = nodes; end
    end
    rate = P.rateTel * ones(N, 1);  rate(tau == 1) = P.rateVid;
    flow = zeros(N, 1);
    for o = find(valid)'
        flow(paths{o}) = flow(paths{o}) + rate(o);
    end
    fac = ones(N, 1);
    for k = find(flow > 0)'
        cap = min(P.nodeCap, P.bw * log2(1 + 10^(Gam(k, serv(k)) / 10)));
        fac(k) = min(1, cap / flow(k));
    end
    deliv = zeros(N, 1);
    for o = find(valid)'
        deliv(o) = rate(o) * min(fac(paths{o}));
    end
    gen  = rate .* alive;
    loss = zeros(N, 1);  loss(alive) = 1 - deliv(alive) ./ rate(alive);
    u = zeros(N, 1);
    u(alive & ~valid) = Ts;
    u(alive & valid & hoNow) = P.Tho;

    % ---- reward, Eq. (4.11) ---------------------------------------------
    wm = 1 + (P.vidMult - 1) * tau;
    r = w(1) * deliv - w(2) * wm .* u - w(3) * wm .* loss ...
        - w(4) * nSig - w(5) * rho - w(6) * nAtt;
    r(~alive) = 0;
    Jtot = Jtot + sum(r) / N;

    % ---- metrics --------------------------------------------------------
    sumU = sumU + sum(u);  sumDel = sumDel + sum(deliv);  sumGen = sumGen + sum(gen);
    sumRho = sumRho + sum(rho);  sumSig = sumSig + sum(nSig);
    validCnt = validCnt + sum(valid);  aliveCnt = aliveCnt + sum(alive);
    if tau(P.videoUAV) == 1 && alive(P.videoUAV)
        vidDel = vidDel + deliv(P.videoUAV);  vidGen = vidGen + rate(P.videoUAV);
        vidSlots = vidSlots + 1;
    end
    rel = find(flow - rate .* valid > 1e-9);
    if numel(rel) >= 2
        lv = flow(rel) / P.nodeCap;  lvSum = lvSum + var(lv, 1);  lvCnt = lvCnt + 1;
    end
    SERV(:, t) = serv;
    for i = 1:N, if alive(i), GS(i, t) = Gam(i, serv(i)); end, end

    % ---- energy update --------------------------------------------------
    E(alive) = E(alive) - P.eIdle - P.eFwd * flow(alive) - P.eHO * nAtt(alive);
    died = alive & (E <= 0);
    nDeaths = nDeaths + sum(died);
    alive(died) = false;
    flowPrev = flow;
end

nTot = max(sumGen, 1e-9);
R.J        = Jtot;
R.interrupt= sumU / N;                       % mean interruption time per UAV [s]
R.ppr      = sumRho / max(nHOok, 1);         % ping-pong rate
R.pdr      = sumDel / nTot;                  % packet delivery ratio
R.sig      = sumSig / (N * T);               % signalling msgs per UAV-slot
R.ho       = nHOatt / N;                     % handover attempts per UAV
R.hoFail   = nHOfail;
R.vidPDR   = vidDel / max(vidGen, 1e-9);     % delivery ratio of the video stream
R.vidGood  = vidDel / max(vidSlots, 1);      % mean video goodput [Mbps]
R.loadVar  = lvSum / max(lvCnt, 1);          % relay load variance
R.conn     = validCnt / max(aliveCnt, 1);    % fraction of UAV-slots with a path
R.deaths   = nDeaths;                        % UAVs lost to battery depletion
R.SERV = SERV;  R.GS = GS;  R.HOLOG = HOLOG;
end

function serv = init_association(P, Gam)
% Greedy start-up association (same for every policy).
N = P.N; G = P.G;
serv = zeros(N, 1);  attached = false(N, 1);
for pass = 1:N
    for i = 1:N
        if serv(i) ~= 0, continue; end
        if Gam(i, G) >= P.Gmin
            serv(i) = G;  attached(i) = true;
        else
            bg = -Inf;  bj = 0;
            for j = 1:N
                if j ~= i && attached(j) && Gam(i, j) >= P.Gmin && Gam(i, j) > bg
                    bg = Gam(i, j);  bj = j;
                end
            end
            if bj > 0, serv(i) = bj;  attached(i) = true; end
        end
    end
end
serv(serv == 0) = G;
end
