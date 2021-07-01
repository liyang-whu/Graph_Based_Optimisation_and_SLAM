% This script can be used to compare your SLAM system

import minislam.slam.g2o.*;

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();

% Magic tuning for the no-prediction case
parameters.laserDetectionRange = 20;

% -------------------------------------4a--------------------------------
% Setting different gps measurement period
parameters.gpsMeasurementPeriod = 10;

% By setting true / false you can enable different combinations of sensors
parameters.enableGPS = true;
parameters.enableLaser = true;

% Set up the simulator and the output
%simulator = minislam.event_generators.simulation.Simulator(parameters, 'q3-small-develop');
simulator = minislam.event_generators.simulation.Simulator(parameters, 'q3-large-test');

% Create and run the different localization systems
g2oSLAMSystem = G2OSLAMSystem();
results = minislam.mainLoop(simulator, g2oSLAMSystem);


% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')

% -------------------------------------4a--------------------------------
% Plot true and estimated trajectory
minislam.graphics.FigureManager.getFigure("GPS Trajectory in "+ num2str(parameters.gpsMeasurementPeriod)+"s");
clf
% true 
plot(results{1}.vehicleTrueStateHistory(1, :), results{1}.vehicleTrueStateHistory(2, :), 'LineWidth', 1.5)
hold on
% estimated
plot(results{1}.vehicleStateHistory(1, :), results{1}.vehicleStateHistory(2, :))
hold off
legend('True trajectory','Estimated trajectory','Location','southeast');
