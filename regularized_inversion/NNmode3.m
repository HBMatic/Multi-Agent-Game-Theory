clear; clc;

% Number of samples
N = 1000;

% Generate commanded velocities (linearly spaced within limits)
vCmd = linspace(0, 0.22, N);
omegaCmd = linspace(0, 2.84, N);

% Simulate measured (actual) velocities with bias and noise
vActual = 0.95 * vCmd + 0.01 * randn(1, N);
omegaActual = 1.05 * omegaCmd + 0.02 * randn(1, N);

% ---- Data Preparation ----
% Forward Model: maps (vCmd, omegaCmd) -> (vActual, omegaActual)
forwardInput = [vCmd; omegaCmd];
forwardOutput = [vActual; omegaActual];

% Inverse Model: maps (vActual, omegaActual) -> (vCmd, omegaCmd)
inverseInput = [vActual; omegaActual];
inverseOutput = [vCmd; omegaCmd];

%% Train Forward Model Neural Network
forwardNet = fitnet([64, 64], 'trainbr');
forwardNet.performFcn = 'mse';
forwardNet.divideParam.trainRatio = 0.7;
forwardNet.divideParam.valRatio = 0.15;
forwardNet.divideParam.testRatio = 0.15;
[forwardNet, trF] = train(forwardNet, forwardInput, forwardOutput,'useParallel','yes');

%% Train Inverse Model Neural Network
inverseNet = fitnet([64, 64], 'trainbr');
inverseNet.performFcn = 'mse';
inverseNet.divideParam.trainRatio = 0.7;
inverseNet.divideParam.valRatio = 0.15;
inverseNet.divideParam.testRatio = 0.15;
[inverseNet, trI] = train(inverseNet, inverseInput, inverseOutput,'useParallel','yes');

%% Consistency Check
% For a set of commanded values, check if:
%   g(f(x)) ~ x   and   f(g(y)) ~ y.
%
% Pick a test sample x:
testCmd = [0.15; 1.5];  % [v_cmd; omega_cmd]
% Pass through the forward model
predictedMeas = forwardNet(testCmd);
% Now apply the inverse network
reconstructedCmd = inverseNet(predictedMeas);
% Compute consistency error
consistencyErrorCmd = immse(reconstructedCmd, testCmd);

% Similarly, for a test measured value y:
testMeas = [0.14; 1.55];  % [v_actual; omega_actual]
% Pass through the inverse model
predictedCmd = inverseNet(testMeas);
% Now apply the forward network
reconstructedMeas = forwardNet(predictedCmd);
% Compute consistency error
consistencyErrorMeas = immse(reconstructedMeas, testMeas);

fprintf('Consistency error for commanded velocities: %f\n', consistencyErrorCmd);
fprintf('Consistency error for measured velocities: %f\n', consistencyErrorMeas);