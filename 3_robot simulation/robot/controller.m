classdef controller < handle
    properties
        PD_like_Kp = [100; 80];
        PD_like_Kd = [30; 30];        
    end
    
    methods
        function this = controller()
            
        end
        
        function Initial(this)

        end
        
        function u = ComputeTorque(this, robot, pcmd, varargin)
            u = this.PD_like_Kd .* (this.PD_like_Kp .* (pcmd - robot.q) - robot.q_dot);
        end
        
    end
end