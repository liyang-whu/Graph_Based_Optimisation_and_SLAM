% This script can be used to develop your SLAM system. It runs a "little
% map" which should be a lot faster to work with

import minislam.slam.g2o.*;

% Configure to disable unneed sensors
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;
parameters.enableLaser = true;

% This is how to change the covariance in the simulated laser outputs
parameters.RLaser(2,2) = (20*pi/180)^2;

% Development or test mode?
% scenarioDirectory = 'q3-small-develop';
scenarioDirectory = 'q3-large-test';

parameters.perturbWithNoise = true;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, scenarioDirectory);

% Create and run the different localization systems
g2oSLAMSystem = G2OSLAMSystem();
results = minislam.mainLoop(simulator, g2oSLAMSystem);

% You will need to add your analysis code here

% Here's how to print out the number of edges and vertices
g2oGraph = g2oSLAMSystem.optimizer();

numVertices = length(g2oGraph.vertices())
numEdges = length(g2oGraph.edges())

landmarkvertex=0;
vertices = g2oGraph.vertices();
for i=1:numVertices
    if (class(vertices{i}) == "minislam.slam.g2o.LandmarkVertex")
        landmarkvertex = landmarkvertex+1;
    end
end

landmarkvertex
vehiclevertices = numVertices - landmarkvertex

observations=0;
edges = g2oGraph.edges();
for i=1:numVertices
    if (class(edges{i}) == "minislam.slam.g2o.LandmarkMeasurementEdge")
        observations = observations+1;
    end
end

observations
landobs = observations/landmarkvertex

