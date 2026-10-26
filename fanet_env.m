function ENV = fanet_env(P, seed, scen)
%FANET_ENV  Pre-compute everything that does not depend on handover decisions.
%   Because UAV mobility is scripted (assumption A1), positions, SINR,
%   predicted link lifetime and predictor confidence can be generated once
%   per (seed, scenario) and reused by every policy.  This guarantees all
%   policies are compared on exactly the same channel realisation.
%
%   scen : 'nominal' | 'jam' | 'fail'
%   ENV fields
%     POS  (N+1 x 3 x T)  node positions (GCS is node N+1)
%     GAM  (N+1 x N+1 x T) SINR in dB, symmetric, -Inf on the diagonal
%     LH   (N+1 x N+1 x T) predicted link lifetime  L_hat  [slots]
%     KAP  (N+1 x N+1 x T) predictor confidence kappa in [0,1]
%     E0   (N x 1)         initial battery
fanet_seed(seed);
N = P.N; G = P.G; T = P.T; Ts = P.Ts; L = P.Lmax;

% ---- 1. scripted lawnmower paths + wind drift -------------------------
W = P.fieldW / N;                         % strip width per UAV
POS = zeros(N + 1, 3, T);
VEL = zeros(N + 1, 3, T);
for i = 1:N
    x0 = (i - 1) * W + 10;  x1 = i * W - 10;
    lanes = x0:P.laneSpacing:x1;
    wp = [];
    for k = 1:numel(lanes)
        if mod(k, 2) == 1
            wp = [wp; lanes(k) 0; lanes(k) P.fieldW];
        else
            wp = [wp; lanes(k) P.fieldW; lanes(k) 0];
        end
    end
    wp  = [wp; wp(1, :)];                 % close the loop
    seg = sqrt(sum(diff(wp).^2, 2));
    cum = [0; cumsum(seg)];
    s0  = rand * cum(end);                % random phase along the path
    s   = mod(s0 + P.speed * Ts * (0:T), cum(end));
    px  = interp1(cum, wp(:, 1), s);
    py  = interp1(cum, wp(:, 2), s);
    VEL(i, 1:2, :) = reshape([diff(px); diff(py)] / Ts, 1, 2, T);
    % wind drift (AR(1)), not known to the predictor
    wx = zeros(1, T); wy = zeros(1, T);
    for t = 2:T
        wx(t) = P.windRho * wx(t-1) + sqrt(1 - P.windRho^2) * P.windStd * randn;
        wy(t) = P.windRho * wy(t-1) + sqrt(1 - P.windRho^2) * P.windStd * randn;
    end
    POS(i, 1, :) = reshape(px(1:T) + wx, 1, 1, T);
    POS(i, 2, :) = reshape(py(1:T) + wy, 1, 1, T);
    POS(i, 3, :) = P.alt;
end
POS(G, 1, :) = P.gcs(1); POS(G, 2, :) = P.gcs(2); POS(G, 3, :) = P.gcs(3);

% ---- 2. node pairs -----------------------------------------------------
[pi_, pj_] = find(triu(ones(N + 1), 1));  % all i<j pairs (np x 1)
np = numel(pi_);

% ---- 3. temporally correlated log-normal shadowing per pair -------------
X = zeros(np, T);
X(:, 1) = P.sigmaX * randn(np, 1);
for t = 2:T
    X(:, t) = P.rhoX * X(:, t-1) + sqrt(1 - P.rhoX^2) * P.sigmaX * randn(np, 1);
end

% ---- 4. SINR, model SINR, look-ahead prediction -------------------------
GAM = -Inf(N + 1, N + 1, T);
LH  = zeros(N + 1, N + 1, T);
KAP = zeros(N + 1, N + 1, T);
N0  = 10^(P.N0_dBm / 10);
useJam = strcmp(scen, 'jam');
sl = (1:L) * Ts;                          % 1 x L look-ahead times
for t = 1:T
    Pt = squeeze(POS(:, :, t));           % (N+1) x 3
    Vt = squeeze(VEL(:, :, min(t, T)));   % (N+1) x 3
    if size(Vt, 2) ~= 3, Vt = reshape(Vt, N + 1, 3); end
    A = Pt(pi_, :); B = Pt(pj_, :);
    PLm = fanet_pathloss(P, A, B);        % model (mean) path loss
    I   = zeros(np, 1);
    if useJam && t >= P.jamSlots(1) && t <= P.jamSlots(2)
        I = jam_power(P, A, B);
    end
    gam_meas = P.Ptx_dBm - (PLm + X(:, t)) - 10 * log10(N0 + I);
    gam_mod  = P.Ptx_dBm - PLm - 10 * log10(N0);        % predictor ignores jammer
    % --- look-ahead with constant-velocity extrapolation ---
    Va = Vt(pi_, :); Vb = Vt(pj_, :);
    Ax = reshape(A(:, 1) + Va(:, 1) * sl, [], 1);
    Ay = reshape(A(:, 2) + Va(:, 2) * sl, [], 1);
    Bx = reshape(B(:, 1) + Vb(:, 1) * sl, [], 1);
    By = reshape(B(:, 2) + Vb(:, 2) * sl, [], 1);
    Az = repmat(A(:, 3), L, 1);  Bz = repmat(B(:, 3), L, 1);
    PLf = reshape(fanet_pathloss(P, [Ax Ay Az], [Bx By Bz]), np, L);
    gam_f = P.Ptx_dBm - PLf - 10 * log10(N0);
    predG = repmat(gam_meas, 1, L) + (gam_f - repmat(gam_mod, 1, L));
    below = predG < P.Gmin;
    [hasB, firstIdx] = max(double(below), [], 2);
    Lh = firstIdx;  Lh(hasB == 0) = L;
    kap = exp(-abs(gam_meas - gam_mod) / P.sigRef);
    for q = 1:np
        i = pi_(q); j = pj_(q);
        GAM(i, j, t) = gam_meas(q); GAM(j, i, t) = gam_meas(q);
        LH(i, j, t)  = Lh(q);       LH(j, i, t)  = Lh(q);
        KAP(i, j, t) = kap(q);      KAP(j, i, t) = kap(q);
    end
end
ENV.POS = POS; ENV.GAM = GAM; ENV.LH = LH; ENV.KAP = KAP;
ENV.E0  = P.E0range(1) + diff(P.E0range) * rand(N, 1);
ENV.scen = scen; ENV.seed = seed;
end

function I = jam_power(P, A, B)
% Interference (mW) at a link: worst of the two endpoints (symmetric).
Jp = P.jamPos;
dA = max(sqrt(sum((A - repmat(Jp, size(A, 1), 1)).^2, 2)), 1);
dB = max(sqrt(sum((B - repmat(Jp, size(B, 1), 1)).^2, 2)), 1);
dm = min(dA, dB);
I  = 10 .^ ((P.jamPow - P.PL0 - 10 * P.eta * log10(dm)) / 10);
end
