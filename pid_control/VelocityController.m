classdef VelocityController
    properties
        % Final Tuned PID Gains
        Kp_v = 1.5;  % Increased Proportional gain for faster response
        Ki_v = 0.1;  % Stronger Integral gain to remove steady-state error
        Kd_v = 0.1;  % Kept same

        Kp_w = 1.2;  % Kept same
        Ki_w = 0.01; % Kept same
        Kd_w = 1.0;  % Increased damping for low-speed stability

        % Error Storage
        prev_e_v = 0; prev_e_w = 0;% Previous errors
        prev_v = 0; prev_w = 0;% Previous values
        integral_e_v = 0; integral_e_w = 0; % Integral accumulations

        % Velocity Limits
        % max_linear_speed = 0.22;  
        % max_angular_speed = 2.84; 
        max_linear_speed = 0.15;  
        max_angular_speed = 0.15;
    end
    
    methods
        function obj = VelocityController(Kp_v, Ki_v, Kd_v, Kp_w, Ki_w, Kd_w)
            if nargin > 0
                obj.Kp_v = Kp_v; obj.Ki_v = Ki_v; obj.Kd_v = Kd_v;
                obj.Kp_w = Kp_w; obj.Ki_w = Ki_w; obj.Kd_w = Kd_w;
            end
        end
        
        function [u_v, u_w, obj] = step(obj, v_d, w_d, v_actual, w_actual, dt)
            % Apply Stronger Low-Pass Filtering to Reduce Noise
            % alpha = 0.85;
            alpha = 0.9;
            v_actual_filtered = alpha * obj.prev_v + (1 - alpha) * v_actual;
            w_actual_filtered = alpha * obj.prev_w + (1 - alpha) * w_actual;

            % Compute velocity errors
            e_v = v_d - v_actual_filtered;  
            e_w = w_d - w_actual_filtered;  
            % e_v = v_d- v_actual;
            % e_w = w_d- w_actual;

            % PID Control for Linear Velocity (with feedforward)
            obj.integral_e_v = obj.integral_e_v + e_v * dt;
            derivative_e_v = (e_v - obj.prev_e_v) / dt;
            u_v = 0.5 * v_d + (obj.Kp_v * e_v + obj.Ki_v * obj.integral_e_v + obj.Kd_v * derivative_e_v);

            % PID Control for Angular Velocity
            obj.integral_e_w = obj.integral_e_w + e_w * dt;
            derivative_e_w = (e_w - obj.prev_e_w) / dt;
            u_w = obj.Kp_w * e_w + obj.Ki_w * obj.integral_e_w + obj.Kd_w * derivative_e_w;

            % Apply velocity limits
            u_v = max(min(u_v, obj.max_linear_speed), -obj.max_linear_speed);
            u_w = max(min(u_w, obj.max_angular_speed), -obj.max_angular_speed);

            % Store previous errors for next iteration
            % obj.prev_v = v_actual_filtered;
            % obj.prev_w = w_actual_filtered;
            obj.prev_e_v = v_actual_filtered;
            obj.prev_e_w = w_actual_filtered;
            % obj.prev_e_v = e_v;
            % obj.prev_e_w = e_w;    
        end
    end
end