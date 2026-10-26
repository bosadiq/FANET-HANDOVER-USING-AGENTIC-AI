FANET Handover: An Agentic AI Framework for Agricultural Surveillance UAV Network Swarms

MATLAB simulation of the framework

fanet_matlab
train_policy        % optional: re-train the policy (cross-entropy method) to get the theta_trained.mat'
main_fanet          % evaluates all methods, writes results.csv and plot all figures figures


main_fanet.m uses the stored theta_trained.mat. Set doTrain = true at the top to retrain.

 fanet_params.m every parameter (field, channel, traffic, weights, thresholds)
 fanet_pathloss.m log-distance path loss plus exact 3-D obstacle blockage (barn, tree line)
 fanet_env.m scripted lawnmower paths with wind drift, correlated shadowing, SINR, link-lifetime predictor and confidence 
 fanet_run.m  slot-by-slot Dec-POMDP: observations, handover execution delay, make-before-break, goodput, loss, interruption, energy, team reward 
 fanet_decide.m the policies: RSSI, A3, and the agent (utility policy, constraint masks C2-C4, confidence gate with A3 fallback) 
 fanet_chain.m serving-chain helper (loop check, hop count) 
 fanet_train.m cross-entropy-method training with parameter sharing 
 main_fanet.m experiments, tables, figures 

## The agent

Each UAV runs the same policy (parameter sharing). In every slot it:

1. Perceives: SINR to the GCS and neighbours, own state, advertisements (load, energy, hop count).
2. Predicts: link lifetime `L_hat` and confidence `kappa`. The predictor extrapolates positions with
   constant velocity, evaluates the known farm map (barn, tree line) over a 15 s horizon, and applies the
   predicted change to the measured SINR. Confidence falls when the measurement departs from the model
   (shadowing, wind drift, jamming).
3. Decides: scores each candidate (GCS or one of the k strongest neighbours) with
   `u_j = theta . [SINR, lifetime, 1-load, energy, -hops, stay, video x lifetime]` and hands over only if the best
   candidate beats the current link by the learned margin. Candidates must be usable, connected, loop-free,
   and not overloaded or low on energy (C2-C4).
4. Acts: the handover completes after `Dho` slots. The old link stays active meanwhile (make-before-break).
5. Reflects: the reward and outcome counters feed training and the metrics.
6. Fallback: if `kappa < kappa_min` on the serving link, that decision uses the A3 rule instead.

## Methods compared

`RSSI threshold`, `A3 (hysteresis + TTT)`, `Agent (default theta)`, `Agent (trained)`, and three ablations of the
trained agent: `no predictor`, `no adverts` (no load/energy information), `no fallback gate`.
Scenarios: `nominal` (barn and tree-line shadowing plus a scripted video event), `jam` (jammer in the
centre of the field, 150 s to 200 s), `fail` (relay UAV 3 fails at 175 s). Every method sees the identical
channel realisation for each seed.

## Metrics (results.csv)

Interruption time per UAV [s], ping-pong rate, packet delivery ratio, signalling messages per UAV-slot,
handovers per UAV, video delivery ratio, relay load variance, connectivity, UAVs lost to battery, team return J.

## Honest notes and limits

The policy is trained with the cross-entropy method (a black-box policy search over 8 shared parameters),
not MAPPO. It follows the same centralised-training, decentralised-execution idea and needs no toolbox.
For a neural MAPPO agent, wrap fanet_run as an environment (state = fanet_decide observation features,
action = candidate index, reward = per-slot r) and train with the Reinforcement Learning Toolbox or export
the traces to Python (EPyMARL, RLlib).
UAV mobility is scripted (assumption A1), so the channel is computed once per seed and reused by every policy.
The predictor is a model-based look-ahead, not a learned network (assumption A4 asks for a learned f_phi).
Replacing fanet_env.m section 4 with a trained regressor (for example fitrnet) keeps the interface unchanged.
All radio, traffic and battery values are plausible assumptions, not measurements. Change them in
fanet_params.m and state them in the thesis.
Interference from other UAVs is not modelled; only the jammer adds interference.
Compare methods on many seeds (P.eval.seeds) and report mean and spread. Single-seed results vary.


