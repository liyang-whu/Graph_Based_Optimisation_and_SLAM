% This script runs the GPS

import minislam.slam.kalman.*;
import minislam.slam.g2o.*;

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = true;
parameters.enableLaser = false;

% This says how much simulation time happens between each GPS measurement.
% Change as per the coursework instructions
parameters.gpsMeasurementPeriod = 5;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, 'q2');

% Create and run the different localization systems
kalmanFilterSLAMSystem = KalmanFilterSLAMSystem();
g2oSLAMSystem = G2OSLAMSystem();
results = minislam.mainLoop(simulator, {kalmanFilterSLAMSystem, g2oSLAMSystem});

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
plot(results{2}.vehicleCovarianceHistory', '--')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory', 'LineWidth', 2)
hold on
plot(results{2}.vehicleStateHistory'-results{2}.vehicleTrueStateHistory','--', 'LineWidth', 2)
%---------------------------------------Q2.C----------------------------------
% Plot true and estimated trajectory
minislam.graphics.FigureManager.getFigure("GPS Trajectory in "+ num2str(parameters.gpsMeasurementPeriod)+"s");
clf
x = [0,70,100,60,0]; 
y = [0,0,400,40,0]; 
% route 
plot(x, y,'m', 'LineWidth', 3) 
hold on 
% true trajectory
plot(results{1}.vehicleTrueStateHistory(1, :), results{1}.vehicleTrueStateHistory(2, :),'b', 'LineWidth', 1.5) 
hold on 
% estiamted trajectory by Kalman Filter
plot(results{1}.vehicleStateHistory(1, :), results{1}.vehicleStateHistory(2, :),'g', 'LineWidth', 1.5) 
hold on 
% estiamted trajectory by G2O
plot(results{2}.vehicleStateHistory(1, :), results{2}.vehicleStateHistory(2, :),'r', 'LineWidth', 1.5) 
hold off 
legend('route', 'true trajectory', 'estimated by Kalman Filter','estimated by G2O','Location','best'); 