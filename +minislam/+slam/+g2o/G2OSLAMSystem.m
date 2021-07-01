    % This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef G2OSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    properties(Access = protected)
                
        % Flag to run the detailed graph validation checks
        validateGraphOnInitialization;        
        
        % The graph used for performing estimation.
        graph;
        
        % The optimization algorithm
        optimizationAlgorithm;
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarksMap;
        
        % ------------------------Q4.b-----------------------------------
        % Remove all or Remove all but first vehicle edge
        removeOrRemain;
        
        % ------------------------Q4.b-----------------------------------
        % Randomly prune some nodes to simplfy the graph 
        graphPrune;
    end
        
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = G2OSLAMSystem()
            
            % Call the base class constructor
            this = this@minislam.slam.VehicleSLAMSystem();
            
            % Create the graph and the optimization algorithm
            this.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            this.graph.setAlgorithm(algorithm);
            
            % Do detailed checking by default
            this.validateGraphOnInitialization = true;
            
            % Preallocate; this is a lower bound on size
            this.vehicleVertices = cell(1, 10000);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 1;
            
            % Allocate the landmark map
            this.landmarksMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % ------------------------Q4.b-----------------------------------
            % Do you want to try Q4b or not
            % 1 means do Q4b, 0 means don't do it
            this.removeOrRemain = 0;
            
            % ------------------------Q4.c-----------------------------------
            % Do you want to try Q4c or not
            % 1 means do Q4c, 0 means don't do it
            this.graphPrune = 0;
        end
        
        % Get the underlying g2o graph
        function graph = optimizer(this)
            graph = this.graph;
        end
        
        % Set the flag to enable / disable validation. This spots errors in
        % graph construction, but repeated calls can slow things down by
        % 20% or more.
        function setValidateGraph(this, validateGraphOnInitialization)
            this.validateGraphOnInitialization = validateGraphOnInitialization;
        end
        
        function validateGraphOnInitialization = validateGraph(this)
            validateGraphOnInitialization = this.validateGraphOnInitialization;
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. Therefore, this method returns if the
        % localization algorithm thinks optimizing is a good idea. Here we
        % always return false because this gives the fastest results
        % because you just optimize once, right and the very end
        function recommendation = recommendOptimization(this)
           %recommendation = false;
            
            % This is how to do it after every 500 steps
           recommendation = rem(this.stepNumber, 100) == 0;
        end
        
        % ------------------------Q4.b-----------------------------------
        function question4b(this, remain)
            numEdges = this.graph.edges();
            edges_count = length(numEdges);
            disp(["Total edge:",edges_count])
            vehicle_edges_count = 0;
            % remove vehicle edges using for loop
            for i = 1:edges_count
                current_edge = numEdges{i};
                % compare the class of the edge
                if class(current_edge) == "minislam.slam.g2o.VehicleKinematicsEdge"
                    % if the current edge is vehicle edges, count += 1
                    vehicle_edges_count = vehicle_edges_count + 1;
                    % if we want to remain the first edge
                    if (vehicle_edges_count == 1 && remain == 1)
                        disp("Remain first vehicle edge and remove the rest")
                    else
                        %disp(["Remove Number",vehicle_edges_count, "vehicle edges"])
                        this.graph.removeEdge(current_edge);
                    end
                end
            end 
            disp(["vehicle edge:", vehicle_edges_count])
        end
        
        % ------------------------Q4.c-----------------------------------
        function question4c(this, remain)
            % for loop in number of lankmarks
            for i = 1:length(this.landmarksMap)
                current_landmark_edges = this.landmarksMap(i).edges;
                count_edges = length(current_landmark_edges);
                % check if the number of landmark edges is large than the
                % edges we want to remain
                if count_edges > remain
                    % randomly select the edges to delete
                    deleted_edge = randsample(current_landmark_edges,count_edges - remain);
                    count_deleted_edge = length(deleted_edge);
                    for j = 1:count_deleted_edge 
                        deleted = current_landmark_edges{j};
                        this.graph.removeEdge(deleted);                  
                    end
                end
            end
        end
        % This method runs the graph optimiser and the outputs of the
        % optimisation are used as the initial conditions for the next
        % round of the graph. This makes it possible to iterative compute
        % solutions as more information becomes available over time.
        function optimize(this, maximumNumberOfOptimizationSteps)
            % ------------------------Q4.b-----------------------------------
            % Check if we want to do question 4b first
            % 1 means we want to do q4b
            % 0 means we don't want to do q4b
            if (this.removeOrRemain == 1)
                % 1 means we want first vehicle edge remains
                % 0 means we want all vehicle edges removed
                this.question4b(1);
            end
            
            % ------------------------Q4.c-----------------------------------
            % Check if we want to do question 4c first
            % 1 means we want to do q4c
            % 0 means we don't want to do q4c
            if (this.graphPrune == 1)
                % input is the number of edges we want to keep
                this.question4c(2);
            end    
            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            if (nargin > 1)
                this.graph.optimize(maximumNumberOfOptimizationSteps);
            else
                this.graph.optimize();
            end
        end
                
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = robotEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end
        
        function [T, X, P] = robotEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(3, this.vehicleVertexId);
            P = zeros(3, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this) 
            
            landmarkVertices = values(this.landmarksMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(2, numberOfLandmarks);
            P = NaN(2, 2, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
    end
        
    % These are the methods you will need to overload
    methods(Access = protected)
                        
        % Declare bodies all of these methods. This makes it possible to
        % instantiate this object.
         
        function handleInitialConditionEvent(this, event)
            
            % Add the first vertex and the initial condition edge
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data)
            this.currentVehicleVertex.setFixed(true);
            this.graph.addVertex(this.currentVehicleVertex);
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
       end
       
        function handleNoPrediction(~)
          %Nothing to do here
        end
        
        function handlePredictToTime(this, time, dT)
            % Create the next vehicle vertex and add it to the graph
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(time);
            this.graph.addVertex(this.currentVehicleVertex);
            
            %Defining the current vehicle vertex state vector
            priorX = this.vehicleVertices{this.vehicleVertexId}.estimate;
            
            % Generating an odometry covariance matrix with the same assumption
            % from workshop 04.
            Q = this.uCov;
            omegaQ = inv(Q);

            %Generating a zero-mean additive gaussian noise
            vi = sqrtm(Q) * randn(3,1);
            
            %Implementing the Process Model
            c = cos(priorX(3));
            s = sin(priorX(3));
                 
            %Specifying the Rotation Matrix
            Mi = dT * [c -s 0;
                s c 0;
                0 0 1];
            
            % Defining the new Vertex
            newVertex = priorX + Mi*(this.u+vi);
            % Normlizing the angle between -pi and pi
            newVertex(3) = g2o.stuff.normalize_theta(newVertex(3));
            
            % Creating the edge
            edge = minislam.slam.g2o.VehicleKinematicsEdge(dT);
            %Setting the first vertex of the edge
            edge.setVertex(1,this.vehicleVertices{this.vehicleVertexId}); 
            % Setting the next vertex to the new Vertex and specifying the
            % second vertex of the edge to the new vertex.
            this.currentVehicleVertex.setEstimate(newVertex);
            edge.setVertex(2,this.currentVehicleVertex);
            edge.setMeasurement(this.u);
            edge.setInformation(omegaQ);
            %Adding the edge to the graph
            this.graph.addEdge(edge);
            
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)
            %Creating a GPS measurement edge (unary)
            gps = minislam.slam.g2o.GPSMeasurementEdge();
            gps.setVertex(1,this.vehicleVertices{this.vehicleVertexId});
            % Set the measurement and covariance based on the event
            gps.setMeasurement(event.data);
            gps.setInformation((event.covariance)^(-1));
            % Add the edge to the graph
            this.graph.addEdge(gps);
            % Handle a GPS measurement
        end
        
        function handleLandmarkObservationEvent(this, event)
            
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);
                if (newVertexCreated == true)
                    % when a new landmark vertex is created, we then added
                    % to the graph
                    this.graph.addVertex(landmarkVertex);
                    % Set the initial estimate.
                    landmarkVertex.setEstimate([0;0]);
                end
                % Creating a landmark-vehicle measurement edge (binary)
                landmarkEdge = minislam.slam.g2o.LandmarkMeasurementEdge();
                landmarkEdge.setVertex(1,this.vehicleVertices{this.vehicleVertexId});
                landmarkEdge.setVertex(2,this.landmarksMap(event.landmarkIds(l)));
                % Set the measurement and covariance based on the event
                landmarkEdge.setMeasurement(z)
                % invert whole matrix
                landmarkEdge.setInformation((event.covariance)^(-1));
                % Add the edge to the graph
                this.graph.addEdge(landmarkEdge);
                % error('handleLandmarkObservationEvent: implement me');
                
            end
                
        end
   end
    
    methods(Access = protected)
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)
            
            % If the landmark exists already, return it
            if (isKey(this.landmarksMap, landmarkId) == true)
                landmarkVertex = this.landmarksMap(landmarkId);
                newVertexCreated = false;
                return
            end
            % if not, we create one
            landmarkVertex = minislam.slam.g2o.LandmarkVertex(landmarkId);
            % then newVertexCreated is true
            newVertexCreated = true;
            % then we add the created vertex into the map
            this.landmarksMap(landmarkId) = landmarkVertex;
            % error('createOrGetLandmark: implement me');
            
        end

    end
end
