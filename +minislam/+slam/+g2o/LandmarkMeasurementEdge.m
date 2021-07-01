% Revised from GPSMeasurementEdge, we use binary instead of unary
classdef LandmarkMeasurementEdge < g2o.core.BaseBinaryEdge

    methods(Access = public)
        
        function this = LandmarkMeasurementEdge()
            % We use binary again
            this = this@g2o.core.BaseBinaryEdge(2); 
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();
            % difference between vehicle and landmark estimate
            difference = landmark - x(1:2);
            % range and bearing measurements error
            this.errorZ(1) = norm(difference) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(difference(2), difference(1)) - x(3) - this.z(2));
        end
        
        function linearizeOplus(this)
            x = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();
            % difference between vehicle and landmark estimate
            difference = landmark - x(1:2);
            r = norm(difference);
            % Jacobian
            this.J{1} = [-difference(1)/r -difference(2)/r 0; ...
                difference(2)/(r^2) -difference(1)/(r^2) -1];
            this.J{2} = -this.J{1}(1:2, 1:2); 
        end        
    end
end