function fanet_seed(seed)
%FANET_SEED  Reproducible random numbers in both MATLAB and Octave.
try
    rng(seed, 'twister');
catch
    rand('twister', seed);
    randn('state', seed);
end
end
