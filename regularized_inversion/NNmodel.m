%% Joint Training of Forward and Inverse Models with Adaptive Learning Rate
% This script assumes you have N measured samples.
%   X (N-by-2): inputs [vi, wi]
%   Y (N-by-2): outputs [vo, wo] from your system.
%
% We build two neural networks:
%   netF: Forward network (X -> Y)
%   netI: Inverse network (Y -> X)
%
% The loss function includes:
%   - Forward loss: ||netF(X) - Y||^2
%   - Inverse loss: ||netI(Y) - X||^2
%   - Consistency losses: ||netI(netF(X)) - X||^2 and ||netF(netI(Y)) - Y||^2
%
% The training loop adapts the learning rate every few epochs.

%% Synthetic Data Generation (Replace with Your Measurements)
clc;clear all;
N = 2500;
% Generate N random inputs in the range [-1,1]
X = -1 + 2 * rand(N, 2);  % Each row: [vi, wi]

% Define an invertible forward mapping:
% Example: vo = exp(vi)*cos(wi),  wo = exp(vi)*sin(wi)
Y_vo = exp(X(:,1)) .* cos(X(:,2));
Y_wo = exp(X(:,1)) .* sin(X(:,2));
Y = [Y_vo, Y_wo];

% Convert data to dlarray format (channels x batch)
Xdl = dlarray(X','CB');
Ydl = dlarray(Y','CB');

%% Define the Forward Network (netF) - Maps X -> Y
layersF = [
    featureInputLayer(2, 'Normalization','none','Name','input')
    fullyConnectedLayer(15, 'Name','fc1')
    tanhLayer('Name','tanh1')
    fullyConnectedLayer(2, 'Name','fc2')
    ];
netF = dlnetwork(layerGraph(layersF));

%% Define the Inverse Network (netI) - Maps Y -> X
layersI = [
    featureInputLayer(2, 'Normalization','none','Name','input')
    fullyConnectedLayer(15, 'Name','fc1')
    tanhLayer('Name','tanh1')
    fullyConnectedLayer(2, 'Name','fc2')
    ];
netI = dlnetwork(layerGraph(layersI));

%% Training Settings
numEpochs = 12000;
initialLearningRate = 1e-1;
learningRate = initialLearningRate;  % Starting learning rate
consistencyWeight = 1.0;

% Learning rate decay settings:
decayEpochs = 500;     % Every 250 epochs, update the learning rate.
decayFactor = 0.9;     % Multiply the learning rate by 0.9.

% Initialize Adam optimizer parameters for both networks:
trailingAvgF = [];
trailingAvgSqF = [];
trailingAvgI = [];
trailingAvgSqI = [];
iteration = 0;

%% Training Loop with Adaptive Learning Rate
lossHistory = zeros(numEpochs,1);
for epoch = 1:numEpochs
    iteration = iteration + 1;
    
    % Evaluate loss and gradients via automatic differentiation
    [loss, gradientsF, gradientsI] = dlfeval(@modelLoss, netF, netI, Xdl, Ydl, consistencyWeight);
    
    % Update the forward and inverse networks using Adam optimizer.
    [netF, trailingAvgF, trailingAvgSqF] = adamupdate(netF, gradientsF, ...
        trailingAvgF, trailingAvgSqF, iteration, learningRate);
    [netI, trailingAvgI, trailingAvgSqI] = adamupdate(netI, gradientsI, ...
        trailingAvgI, trailingAvgSqI, iteration, learningRate);
    
    % Adjust the learning rate every 'decayEpochs' epochs
    if mod(epoch, decayEpochs) == 0
        learningRate = learningRate * decayFactor;
        fprintf('Epoch %d: Adjusted Learning Rate = %e\n', epoch, learningRate);
    end
    
    lossHistory(epoch) = double(gather(extractdata(loss)));
    if mod(epoch,100)==0
        fprintf('Epoch %d, Loss = %.5f\n', epoch, lossHistory(epoch));
    end
end

%% Testing the Composite Mapping
% Compute forward predictions and then invert back to the inputs.
Y_pred = predict(netF, Xdl);
X_rec = predict(netI, Y_pred);

% Compute the reconstruction error (mean squared error)
reconstructionError = mean((extractdata(X_rec) - extractdata(Xdl)).^2, 'all');
fprintf('Reconstruction MSE (||netI(netF(X))-X||^2): %.6f\n', reconstructionError);

%% Model Loss Function
function [loss, gradientsF, gradientsI] = modelLoss(netF, netI, Xdl, Ydl, consistencyWeight)
    % Forward prediction using netF: estimate outputs Y_est from inputs Xdl.
    Y_est = forward(netF, Xdl);
    % Inverse prediction using netI: estimate inputs X_est from outputs Ydl.
    X_est = forward(netI, Ydl);
    
    % Compute consistency losses:
    % 1. netI(netF(Xdl)) should reconstruct Xdl.
    X_rec = forward(netI, Y_est);
    % 2. netF(netI(Ydl)) should reconstruct Ydl.
    Y_rec = forward(netF, forward(netI, Ydl));
    
    % Define Mean Squared Error (MSE) losses for each mapping.
    forwardLoss = mean((Y_est - Ydl).^2, 'all');
    inverseLoss = mean((X_est - Xdl).^2, 'all');
    consistencyLossX = mean((X_rec - Xdl).^2, 'all');
    consistencyLossY = mean((Y_rec - Ydl).^2, 'all');
    
    % Total loss is the sum of all losses, with the consistency losses weighted.
    loss = forwardLoss + inverseLoss + consistencyWeight*(consistencyLossX + consistencyLossY);
    
    % Compute gradients with respect to the learnable parameters of netF and netI.
    gradientsF = dlgradient(loss, netF.Learnables);
    gradientsI = dlgradient(loss, netI.Learnables);
end
