%MAIN_FANET  Run the full study: train, evaluate, tabulate, plot.
%   Agentic AI network agents for FANET handover in agricultural surveillance.
%   
%   Set doTrain = false to reuse theta_trained.mat
clear; clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
doTrain   = false;     % true: re-run CEM training (a few minutes)
makePlots = true;
P = fanet_params();

% ---- 1. policy parameters ----------------------------------------------
tfile = fullfile(here, 'theta_trained.mat');
if doTrain || ~exist(tfile, 'file')
    fprintf('Training the shared agent policy (cross-entropy method)...\n');
    [thetaT, trainHist] = fanet_train(P, true);
    save(tfile, 'thetaT', 'trainHist');
else
    S = load(tfile);  thetaT = S.thetaT;
    fprintf('Loaded trained policy from theta_trained.mat\n');
end
fprintf('theta_trained = %s\n\n', mat2str(thetaT, 3));

% ---- 2. methods ----------------------------------------------------------
M = {  'RSSI threshold',          'rssi',         P.theta0;
       'A3 (hyst. + TTT)',        'a3',           P.theta0;
       'Agent (default theta)',   'agent',        P.theta0;
       'Agent (trained)',         'agent',        thetaT;
       'Agent - no predictor',    'agent_nopred', thetaT;
       'Agent - no adverts',      'agent_noads',  thetaT;
       'Agent - no fallback gate','agent_nogate', thetaT };
scens  = {'nominal', 'jam', 'fail'};
seeds  = P.eval.seeds;
fld    = {'interrupt','ppr','pdr','sig','ho','vidPDR','loadVar','conn','deaths','J'};
nM = size(M, 1);  nS = numel(scens);  nK = numel(seeds);
RES = zeros(nM, numel(fld), nS, nK);

% ---- 3. evaluation (same channel realisation for every method) -------------
for c = 1:nS
    fprintf('Scenario: %s\n', scens{c});
    for k = 1:nK
        ENV = fanet_env(P, 1000 + seeds(k), scens{c});
        for m = 1:nM
            R = fanet_run(P, ENV, M{m, 2}, M{m, 3});
            for f = 1:numel(fld), RES(m, f, c, k) = R.(fld{f}); end
        end
    end
end

% ---- 4. results table ----------------------------------------------------
hdr = {'Interrupt[s]','PingPong','PDR','Sig/UAVslot','HO/UAV','VideoPDR','LoadVar','Connect','Deaths','Return J'};
fid = fopen(fullfile(here, 'results.csv'), 'w');
fprintf(fid, 'scenario,method,%s\n', strjoin(hdr, ','));
for c = 1:nS
    fprintf('\n=== %s (mean over %d seeds) ===\n', upper(scens{c}), nK);
    fprintf('%-26s', 'Method');  fprintf('%13s', hdr{:});  fprintf('\n');
    for m = 1:nM
        v = mean(RES(m, :, c, :), 4);
        fprintf('%-26s', M{m, 1});  fprintf('%13.3f', v);  fprintf('\n');
        fprintf(fid, '%s,%s', scens{c}, M{m, 1});  fprintf(fid, ',%.5f', v);  fprintf(fid, '\n');
    end
end
fclose(fid);

% spread over seeds (mean +/- std should be reported in the thesis)
fid = fopen(fullfile(here, 'results_std.csv'), 'w');
fprintf(fid, 'scenario,method,%s\n', strjoin(hdr, ','));
for c = 1:nS
    for m = 1:nM
        v = std(squeeze(RES(m, :, c, :)), 0, 2)';
        fprintf(fid, '%s,%s', scens{c}, M{m, 1});  fprintf(fid, ',%.5f', v);  fprintf(fid, '\n');
    end
end
fclose(fid);

% ablation summary: change of each ablation relative to the full trained agent
ab = [5 6 7];                                   % rows of M that are ablations
fprintf('\n=== ABLATION: change relative to Agent (trained), mean over %d seeds ===\n', nK);
fid = fopen(fullfile(here, 'results_ablation.csv'), 'w');
fprintf(fid, 'scenario,variant,%s\n', strjoin(hdr, ','));
for c = 1:nS
    fprintf('\n%s\n%-26s', upper(scens{c}), 'Variant');  fprintf('%13s', hdr{:});  fprintf('\n');
    ref = mean(RES(4, :, c, :), 4);
    for m = ab
        d = mean(RES(m, :, c, :), 4) - ref;     % absolute change vs full agent
        fprintf('%-26s', M{m, 1});  fprintf('%13.3f', d);  fprintf('\n');
        fprintf(fid, '%s,%s', scens{c}, M{m, 1});  fprintf(fid, ',%.5f', d);  fprintf(fid, '\n');
    end
end
fclose(fid);
fprintf('(positive Interrupt/PingPong/Sig/HO/LoadVar = worse; negative PDR/VideoPDR = worse)\n');
save(fullfile(here, 'results.mat'), 'RES', 'M', 'fld', 'scens', 'seeds');
fprintf('\nSaved results.csv and results.mat\n');

% ---- 5. figures ----------------------------------------------------------
if makePlots
    cRed = [0.84 0.15 0.16];  cBlue = [0.12 0.47 0.71];  cGreen = [0.17 0.63 0.17];
    cOrange = [1.00 0.50 0.05];  cPurple = [0.58 0.40 0.74];
    sel = [1 2 4];  col = [cRed; cBlue; cGreen];     % RSSI = red, A3 = blue, Agent = green
    pick = {'interrupt','Interruption time per UAV [s]';
            'pdr','Packet delivery ratio';
            'ppr','Ping-pong rate';
            'vidPDR','Video delivery ratio'};
    f1 = figure('Position', [50 50 1000 700], 'Color', 'w');
    for p = 1:4
        subplot(2, 2, p);
        fi = find(strcmp(fld, pick{p, 1}));
        Y = zeros(nS, numel(sel));
        for c = 1:nS, Y(c, :) = mean(RES(sel, fi, c, :), 4)'; end
        b = bar(Y);
        for q = 1:numel(sel), set(b(q), 'FaceColor', col(q, :)); end
        set(gca, 'XTickLabel', scens);  ylabel(pick{p, 2});  grid on;
        if p == 1, legend(M(sel, 1), 'Location', 'northwest'); end
    end
    print(f1, fullfile(here, 'fig_metrics.png'), '-dpng', '-r150');

    % ---- ablation figure: full agent vs. variants with one component removed ----
    abIdx = [4 5 6 7];
    shade = [cGreen; cBlue; cOrange; cPurple];       % full agent = green (as in Fig. 5.2)
    abPick = {'interrupt','Interruption time per UAV [s]';
              'pdr','Packet delivery ratio';
              'vidPDR','Video delivery ratio';
              'ppr','Ping-pong rate';
              'loadVar','Relay load variance';
              'ho','Handovers per UAV'};
    f4 = figure('Position', [50 50 1200 700], 'Color', 'w');
    for p = 1:6
        subplot(2, 3, p);
        fi = find(strcmp(fld, abPick{p, 1}));
        Y = zeros(nS, numel(abIdx));
        for c = 1:nS, Y(c, :) = mean(RES(abIdx, fi, c, :), 4)'; end
        b = bar(Y);
        for q = 1:numel(abIdx), set(b(q), 'FaceColor', shade(q, :)); end
        set(gca, 'XTickLabel', scens);  ylabel(abPick{p, 2});  grid on;
        if p == 1
            yl = ylim;  ylim([0 yl(2) * 1.5]);
            legend(M(abIdx, 1), 'Location', 'northwest');
        end
    end
    print(f4, fullfile(here, 'fig_ablation.png'), '-dpng', '-r150');

    ENV = fanet_env(P, 1001, 'nominal');
    Ra = fanet_run(P, ENV, 'a3', P.theta0);
    Rg = fanet_run(P, ENV, 'agent', thetaT);
    f2 = figure('Position', [50 50 1000 650], 'Color', 'w');
    u = P.videoUAV;  tt = (1:P.T) * P.Ts;
    for p = 1:2
        subplot(2, 1, p);  R = Ra;  nm = 'A3 baseline';
        if p == 2, R = Rg; nm = 'Agent (trained)'; end
        plot(tt, R.GS(u, :), '-', 'Color', cBlue, 'LineWidth', 1.3); hold on;
        plot(tt, P.Gmin * ones(size(tt)), '--', 'Color', cRed, 'LineWidth', 1.2);
        ev = R.HOLOG(R.HOLOG(:, 2) == u, 1);
        for q = 1:numel(ev), plot(ev(q) * P.Ts * [1 1], ylim, ':', 'Color', cGreen, 'LineWidth', 1.3); end
        yl = ylim;  patch(P.videoSlots([1 2 2 1]) * P.Ts, yl([1 1 2 2]), cOrange, ...
                          'FaceAlpha', 0.18, 'EdgeColor', 'none');
        xlabel('Time [s]');  ylabel(sprintf('SINR of UAV %d serving link [dB]', u));
        title(sprintf('%s   (dotted = handover completed, shaded = video event)', nm));
        grid on;
    end
    print(f2, fullfile(here, 'fig_timeline.png'), '-dpng', '-r150');

    t0 = 250;  f3 = figure('Position', [50 50 700 700], 'Color', 'w');  hold on;
    obstCol = [0.85 0.60 0.35; 0.55 0.80 0.45];      % barn = brown, tree line = green
    for o = 1:size(P.obst, 1)
        ob = P.obst(o, :);
        rectangle('Position', [ob(1) ob(3) ob(2)-ob(1) ob(4)-ob(3)], ...
                  'FaceColor', obstCol(min(o, 2), :), 'EdgeColor', [0.3 0.3 0.3]);
    end
    Pt = squeeze(ENV.POS(:, :, t0));
    plot(Pt(P.G, 1), Pt(P.G, 2), 's', 'MarkerSize', 14, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', cRed);
    text(Pt(P.G, 1) + 15, Pt(P.G, 2) + 25, 'GCS');
    for i = 1:P.N
        plot(Pt(i, 1), Pt(i, 2), 'o', 'MarkerSize', 9, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', cBlue);
        text(Pt(i, 1) + 10, Pt(i, 2) + 15, sprintf('UAV %d', i));
        s = Rg.SERV(i, t0);
        plot([Pt(i, 1) Pt(s, 1)], [Pt(i, 2) Pt(s, 2)], '-', 'Color', cBlue, 'LineWidth', 1.4);
    end
    axis equal;  axis([0 P.fieldW 0 P.fieldW]);  grid on;  box on;
    xlabel('x [m]');  ylabel('y [m]');
    title(sprintf('Serving links chosen by the agents at t = %.0f s  (brown = barn, green = tree line)', t0 * P.Ts));
    print(f3, fullfile(here, 'fig_topology.png'), '-dpng', '-r150');
    fprintf('Saved fig_metrics.png, fig_ablation.png, fig_timeline.png, fig_topology.png\n');
end
