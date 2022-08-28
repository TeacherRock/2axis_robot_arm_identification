classdef SCARA < handle
    properties
        Beta, par
        ax = 2
        
        sz = [2, 1]
    end
    
    methods (Access = public)
        function this = SCARA()
            [this.Beta, this.par ] = this.Parameter;
            
        end
        function [ M_Full ] = M(this, P1)
           [ M_Full ] = this.M_Full(P1, this.Beta); 
        end
        function [ N_Full ] = N(this, P1, V1)
           [ N_Full ] = this.N_Full(P1, V1, this.Beta); 
        end
        function [ W_Full ] = W( P1)
           [ W_Full ] = this.W_Full(P1, V1, A1); 
        end
    end
    
    methods (Access = private)
        [ Beta, par ] = Parameter( this );
        
    end
    
end