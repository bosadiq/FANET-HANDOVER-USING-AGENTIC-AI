function [theta, hist] = fanet_train(P, verbose)
%FANET_TRAIN  Train the shared agent policy with the cross-entropy method.
%   Centralised training, decentralised execution: all UAVs share theta
%   (parameter sharing); each candidate theta is scored by the team return
%   J averaged over training seeds and scenarios, then the mean
%   and std of the sampling distribution move to the elite samples.
%   Needs no toolbox.  A MAPPO version needs the Reinforcement Learning
%   Toolbox (see README, "Extending to MAPPO").
if nargin < 2, verbose = true; end
tr = P.train;
envs = {};
for s = tr.seeds
    for c = 1:numel(tr.scens)
        envs{end + 1} = fanet_env(P, s, tr.scens{c}); %#ok<AGROW>
    end
end
fanet_seed(2024);
D  = numel(P.theta0);
mu = P.theta0;
sg = tr.sigma0 * ones(1, D);
hist = zeros(tr.iters, 3);
for it = 1:tr.iters
    pop = repmat(mu, tr.pop, 1) + randn(tr.pop, D) .* repmat(sg, tr.pop, 1);
    pop(1, :) = mu;                                   % always keep the current mean
    pop(:, 1:7) = min(max(pop(:, 1:7), 0), 5);        % weights in [0, 5]
    pop(:, 8)   = min(max(pop(:, 8),   0), 3);        % hysteresis margin in [0, 3]
    score = zeros(tr.pop, 1);
    for q = 1:tr.pop
        for e = 1:numel(envs)
            R = fanet_run(P, envs{e}, 'agent', pop(q, :));
            score(q) = score(q) + R.J / numel(envs);
        end
    end
    [~, ord] = sort(score, 'descend');
    el = pop(ord(1:tr.elite), :);
    mu = 0.5 * mu + 0.5 * mean(el, 1);
    sg = max(0.5 * sg + 0.5 * std(el, 0, 1), 0.05);
    hist(it, :) = [it, score(ord(1)), mean(score)];
    if verbose
        fprintf('CEM iter %2d/%d   best J = %8.1f   mean J = %8.1f\n', ...
                it, tr.iters, score(ord(1)), mean(score));
    end
end
theta = pop(ord(1), :);                               % best sample of the last iteration
end
