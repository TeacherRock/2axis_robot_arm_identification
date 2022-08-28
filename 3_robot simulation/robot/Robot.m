classdef Robot < handle
    properties
       q, q_dot, q_ddot, torque 
    end
    
    properties(Access = private)
        % Real data
        
        
        cr  %  controller
        rb  %  robot
        
        ts 
        par
        sz
    end
    
    methods
        function this = Robot(ts, rb)
            this.rb = eval([rb, '()']);
            this.sz = [this.rb.sz, 1];
            [this.q, this.q_dot, this.q_ddot, this.torque] = deal(zeros(this.sz));
            this.ts = ts;
            this.par = this.rb.par;
        end
        
        % 給定軌跡進行追蹤
        function res = Tracking(this, cmd, cr)
            % Given Command in cmd.length, samplingtime, t, pos, vel, acc
            
            % Get Parameter -----------------------------------------------
            ax = this.rb.ax;
            Ts = this.ts;
            % Get Parameter -----------------------------------------------
            
            % Initial -----------------------------------------------------
            this.StrDisp('Start Initial'); tic
            % >>>> Create Space
            res.t = Ts : Ts : cmd.length * cmd.samplingtime;
            L = length(res.t);
            
            [res.q, res.q_dot, res.q, res.torque] = deal(zeros(ax, L));
                        
            % >>>> System Initial 
            this.initial(cmd.pos(1, :)');
            cr.Initial();
                        
            this.StrDisp(['End Initial in ', num2str(toc), ' sec']);

            % Tracking ----------------------------------------------------
            this.StrDisp('Start Tracking'); tic
            index = 1;
            
            for t = Ts : Ts : cmd.length * cmd.samplingtime
                i = (floor(t/cmd.samplingtime));
                % >>>> get command
                if i > 0
                    pcmd = cmd.pos(i,:)'; 
                    vcmd = cmd.vel(i,:)'; 
                    acmd = cmd.acc(i,:)'; 
                else
                    pcmd = cmd.pos(1,:)';
                    [vcmd, acmd] = deal(zeros(this.rb.ax, 1));
                end
                
                % >>>> calculate controller torque
                this.torque = cr.ComputeTorque(this, pcmd, vcmd, acmd);
                
                % >>>> calculate state
                this.q_ddot = this.rb.M( this.q ) \ (this.torque - this.rb.N(this.q, this.q_dot) );
                this.q_dot  = this.q_dot + this.q_ddot .* this.ts;
                this.q      = this.q     + this.q_dot  .* this.ts;
                
                
                % >>>> record
                res.q     (:, index) = this.q     ;
                res.q_dot (:, index) = this.q_dot ;
                res.q_ddot (:, index) = this.q_ddot ;
                res.torque(:, index) = this.torque;
                
                index = index + 1;
            end
            this.StrDisp(['End Tracking in ', num2str(toc), ' sec']) ; disp(' ');
            % Tracking ----------------------------------------------------
            
            res.cr = cr;
            res.cmd = cmd;
        end
             
    end
    
    methods (Access = private)
        % 模擬初始化
        function initial(this, MotorPos)
           
           [this.q_dot, this.q_ddot] = deal(zeros(this.rb.ax,1));
           
           this.q = MotorPos;
           
        end
        
        function StrDisp(~, str)
            res = '>> -----------------------------------------------------------------------';
            res(4 : 3 + length(str) + 1) = [str, ' '];
            
            disp(res)
        end
    end
    
    
end
