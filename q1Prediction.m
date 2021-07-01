% This script runs the odometry

import minislam.slam.kalman.*;
import minislam.slam.g2o.*;

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;
parameters.enableLaser = false;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
parameters.perturbWithNoise = true;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, 'q1');

% Create and run the different localization systems
kalmanFilterSLAMSystem = KalmanFilterSLAMSystem();
g2oSLAMSystem = G2OSLAMSystem();
results = minislam.mainLoop(simulator, {kalmanFilterSLAMSystem, g2oSLAMSystem});

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times (s)');
clf
plot(results{1}.optimizationTimes)
hold on
plot(results{2}.optimizationTimes,'+')
xlabel("timestep");
ylabel("Optimisation Times (s)");


% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
plot(results{2}.vehicleCovarianceHistory', '--')
legend("x-covariance (Kalman)","y-covariance (Kalman)","heading covariance (Kalman)",...
    "x-covariance (g2o)","y-covariance (g2o)","heading covariance (g2o)", 'Location','northwest');
xlabel("timestep");
ylabel("Covariance");

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
xlabel("timestep");
ylabel("Error");
title("Errors (SLAM system Trajectory - True Trajectory)");
hold on
plot(results{2}.vehicleStateHistory'-results{2}.vehicleTrueStateHistory','--')
legend("x-error (Kalman)","y-error (Kalman)","heading error (Kalman)",...
    "x-error (g2o)","y-error (g2o)","heading error (g2o)", 'Location','northwest');
%%
%------------------------------Q1.C-------------------------------------
% Plot g2o and kalman filter trajectory   
minislam.graphics.FigureManager.getFigure('Trajectory of Kalman Filter and G2O'); 
clf
% x and y data for route
x = [0,70,70,0,0]; 
y = [0,0,70,70,0]; 
% route 
plot(x, y,'black', 'LineWidth', 3) 
hold on 
% true trajectory
plot(results{1}.vehicleTrueStateHistory(1, :), results{1}.vehicleTrueStateHistory(2, :),'b', 'LineWidth', 3) 
hold on 
% estiamted trajectory by Kalman Filter
plot(results{1}.vehicleStateHistory(1, :), results{1}.vehicleStateHistory(2, :),'g', 'LineWidth', 1.5) 
hold on 
% estiamted trajectory by G2O
plot(results{2}.vehicleStateHistory(1, :), results{2}.vehicleStateHistory(2, :),'r', 'LineWidth', 1.5) 
hold off 
legend('route', 'true trajectory', 'estimated by Kalman Filter','estimated by g2o','Location','best'); 