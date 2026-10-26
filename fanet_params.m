function P = fanet_params()
%FANET_PARAMS  All simulation parameters (Section 4 notation in comments).
%   Edit values here only; every other file reads this struct.

% ---- Scenario geometry --------------------------------------------------
P.N      = 6;                 % number of UAVs, |U|
P.G      = P.N + 1;           % index of the GCS node G
P.Ts     = 0.5;               % slot duration T_s [s]
P.T      = 600;               % slots per mission (300 s)
P.gcs    = [0 0 5];           % GCS position [x y z] (corner of the field) [m]
P.alt    = 20;                % UAV flight altitude [m]
P.speed  = 10;                % UAV ground speed [m/s]
P.laneSpacing = 40;           % lawnmower lane spacing [m]
P.fieldW = 1000;              % field is fieldW x fieldW [m]
P.windStd = 3;                % std of wind drift around the planned path [m]
P.windRho = 0.95;             % AR(1) coefficient of the wind drift

% Obstacles: [xmin xmax ymin ymax height excessLoss_dB]
P.obst = [ 300 380 150 230 12 20 ;    % barn
           500 520   0 600 30 18 ];   % tree line (windbreak), open gap for y>600

% ---- Channel, Eq. (4.1)-(4.2) -------------------------------------------
P.Ptx_dBm = 26;               % transmit power
P.N0_dBm  = -95;              % noise power
P.PL0     = 40;               % path loss at d0 [dB]
P.d0      = 1;                % reference distance [m]
P.eta     = 2.4;              % path-loss exponent
P.sigmaX  = 3;                % log-normal shadowing std [dB]
P.rhoX    = 0.9;              % temporal correlation of shadowing per slot
P.Gmin    = 5;                % minimum usable SINR  Gamma_min [dB]

% ---- Jammer (scenario 'jam') -------------------------------------------
P.jamPos   = [500 500 2];
P.jamPow   = 30;              % dBm
P.jamSlots = [300 400];

% ---- Traffic and capacity ----------------------------------------------
P.rateTel = 0.5;              % telemetry / imagery [Mbps]
P.rateVid = 4.0;              % video stream [Mbps]
P.videoUAV   = 4;             % UAV that detects the intruder
P.videoSlots = [200 320];     % slots during which tau = 1 for that UAV
P.nodeCap = 5;                % max traffic a UAV can forward [Mbps]
P.bw      = 2;                % link bandwidth for Shannon capacity [MHz]

% ---- Failure (scenario 'fail') -----------------------------------------
P.failUAV  = 3;
P.failSlot = 350;

% ---- Energy ------------------------------------------------------------
P.E0range  = [0.25 1.0];      % initial battery fraction (uniform)
P.eIdle    = 0.0003;          % per-slot idle drain
P.eFwd     = 0.00008;         % extra per-slot drain per Mbps forwarded
P.eHO      = 0.0005;          % drain per handover attempt

% ---- Handover mechanics -------------------------------------------------
P.k       = 3;                % neighbours considered (k strongest)
P.Dho     = 2;                % handover execution delay [slots]
P.Tho     = 0.05;             % residual interruption of a make-before-break [s]
P.Tpp     = 10;               % ping-pong window T_pp [slots]
P.Hmax    = 5;                % maximum hops to GCS
P.lamMax  = 0.9;              % max relay load (constraint C3)
P.Emin    = 0.15;             % min relay energy (constraint C3)
P.msgsPerHO = 3;              % signalling messages per handover attempt

% ---- Predictor ----------------------------------------------------------
P.Lmax    = 30;               % look-ahead horizon [slots]
P.sigRef  = 4;                % dB, scale for confidence kappa = exp(-|err|/sigRef)
P.kapMin  = 0.25;             % confidence threshold kappa_min

% ---- Baselines ----------------------------------------------------------
P.rssiThr = 8;                % RSSI baseline: switch if serving SINR < thr [dB]
P.a3Hys   = 3;                % A3 hysteresis [dB]
P.a3TTT   = 3;                % A3 time-to-trigger [slots]

% ---- Reward weights, Eq. (4.11) ----------------------------------------
%         w1(goodput) w2(interrupt) w3(loss) w4(signal) w5(pingpong) w6(energy)
P.w     = [1.0         4.0          4.0       0.05       0.5          0.1];
P.vidMult = 3;                % video multiplies w2, w3  (w^vid / w^tel)

% ---- Agent policy -------------------------------------------------------
% theta = [w_gamma w_life w_load w_energy w_hop w_stay w_video margin]
P.theta0 = [1.0 1.5 0.5 0.3 0.4 0.3 1.0 0.3];
P.minDwell = 2;               % slots before a voluntary handover is allowed

% ---- Training (cross-entropy method) -----------------------------------
P.train.pop    = 16;
P.train.iters  = 8;
P.train.elite  = 4;
P.train.seeds  = [101 102];
P.train.scens  = {'nominal','jam','fail'};
P.train.sigma0 = 0.5;

% ---- Evaluation ---------------------------------------------------------
P.eval.seeds = 1:10;
end
