%TRAIN_POLICY  Re-train the agent policy and store it in theta_trained.mat.
here = fileparts(mfilename('fullpath'));  addpath(here);
P = fanet_params();
[thetaT, trainHist] = fanet_train(P, true);
save(fullfile(here, 'theta_trained.mat'), 'thetaT', 'trainHist');
fprintf('theta_trained = %s\n', mat2str(thetaT, 3));
