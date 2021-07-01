classdef KalmanFilterSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    % This is a very minimal Kalman filter; its purpose is to let me do
    % some debugging and comparison. It is not fully featured.
    
    properties(Access = protected)
        
        % Kalman filter mean
        xEst;
        PEst;
        
        % Kalman filter covariance
        xPred;
        PPred;
        
        % Store of the mean and covariance values for the vehicle
        timeStore;
        xEstStore;
        PEstStore;
    end
    
    methods(Access = public)
        
        function this = KalmanFilterSLAMSystem()
            this = this@minislam.slam.VehicleSLAMSystem();
            this.xEstStore = NaN(3, 1);
            this.PEstStore = NaN(3, 1);
            this.xEst = NaN(3, 1);
            this.PEst = NaN(3, 3);
        end
        
        function [x,P] = robotEstimate(this)
            x = this.xEst(1:3);
            P = this.PEst(1:3, 1:3);
        end
        
        function [T, X, PX] = robotEstimateHistory(this)
            T = this.timeStore;
            X = this.xEstStore;
            PX = this.PEstStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(this)
            landmarkIds = [];
            x = NaN(2, 0);
            P = NaN(2, 2, 0);
        end
        
        function recommendation = recommendOptimization(this)
            recommendation = true;
        end
        
        function processEvents(this, events)
            % Handle the events
            processEvents@minislam.slam.VehicleSLAMSystem(this, events);
            
            % Store the estimate for the future
            this.timeStore(:, this.stepNumber) = this.currentTime;
            this.xEstStore(:, this.stepNumber) = this.xEst(1:3);
            this.PEstStore(:, this.stepNumber) = diag(this.PEst(1:3, 1:3));
        end
        
        
        function optimize(~, ~)
            % Nothing to do
        end
    end
       
    methods(Access = protected)
                    
        function handleInitialConditionEvent(this, event)
            this.xEst = event.data;
            this.PEst = event.covariance;
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handleNoPrediction(this)
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handlePredictToTime(this, time, dT)

            % You will need to write the code to implement the process
            % model which:
            % 1. Computes this.xPred
            % 2. Computes the Jacobian
            % 3. Computes the process noise
           
            % Rotation
            c = cos(this.xEst(3));
            s = sin(this.xEst(3));
            M = [c -s 0;
                 s  c 0;
                 0  0 1];
            
            % Define the process noise 
            Q = this.uCov;
            vi = sqrtm(Q) * randn(3,1);
            Q_k = diag(vi.^2);     
            
            % Compute the process model like propagation of the state
            this.xPred = this.xEst + dT * M * (this.u + vi);
            
            % Define the jacobian function of the state function
            J_fx = [1  0  -dT * (this.u(1) + vi(1)) * s - dT * vi(2) * c;...
                    0  1   dT * (this.u(1) + vi(1)) * c - dT * vi(2) * s;...
                    0  0   1];
            
            % Define the jacobian function with respect to the process noise
            J_vf = dT * [c -s 0;...
                         s  c 0;...
                         0  0 1];
            
            F_s = eye(size(this.PEst, 1));
            F_s(1:3, 1:3) = J_fx;
            
            B_s = zeros(3, 1);
            B_s(1:3, 1:3) = J_vf;
            
            % Compute the covariance propagation
            this.PPred = F_s .* this.PEst .* F_s.' + B_s .* Q_k .* B_s.';
            
            % Update the state 
            this.xEst = this.xPred;
            this.PEst = this.PPred;
        end
        
        function handleGPSObservationEvent(this, event)
            
            % You will need to write the code to implement the fusing the
            % platform position estimate with a GPS measurement
            % Define the observation matrix
            z = [event.data; this.xPred(3)];
            H_s = eye(3);
            
            % Define the covariance with the GPS measurement
            R = eye(3);
            R(1:2, 1:2) = event.covariance;
            
            % Define the Kalman filter weight (gain)
            W = this.PPred * H_s.' / (H_s * this.PPred * H_s.' + R);
            
            % Update the Kalman Filter state and covariance 
            this.xEst = this.xPred + W * (z -this.xPred);
            this.PEst = this.PPred - W * (H_s * this.PPred * H_s.' + R) * W.';
        end
        
        function handleLandmarkObservationEvent(this, event)
            error('Not supported; this should only be used to implement odometry and GPS')
        end
 
    end
end