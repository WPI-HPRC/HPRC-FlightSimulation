classdef AtmosphericModel
    %AtmosphericModel class stores functions and values to be used with the
    %atmospheric model
    %   Class for the atmospheric model to return density, pressure and
    %   temperature at any altitude
    
    properties
        height
    end

    properties (Constant)
        t_sl = 308.15; % [K] Temperature at sea level
        P_sl = 101325; % [Pa] Pressure at sea level
        a_1 = -6.5E-3; % [K/m] Temperature Gradient
        R = 287; % [J/kgK] Universal Gas Constant
        g = 9.81; % [m/s^2] Gravitational Acceleration on Earth
        rho_sl = 1.225; % [kg/m^3] Density at sea level
    end
    
    methods

        function self = AtmosphericModel(height)
            % ATMOSPHERICMODEL Construct the atmospheric model class
            self.height = height;
        end

        function density = getDensity(self)
            if(self.height <= 11000)
                
            else

            end
            density = self.rho_sl * (self.getTemperature() / self.t_sl) ^ (-1 - (self.g / (self.a_1*self.R)));
        end

        function pressure = getPressure(self) 
            if(self.height <= 11000) 
                
            else 

            end
            pressure = self.P_sl * (self.getTemperature() / self.t_sl) ^ (-self.g / (self.a_1*self.R));
        end

        function temperature = getTemperature(self)
            if(self.height <= 11000)
                
            else

            end
            temperature = self.t_sl + self.a_1*self.height;
        end
        % function obj = untitled(inputArg1,inputArg2)
        %     %UNTITLED Construct an instance of this class
        %     %   Detailed explanation goes here
        %     obj.Property1 = inputArg1 + inputArg2;
        % end
        
        % function outputArg = method1(obj,inputArg)
        %     %METHOD1 Summary of this method goes here
        %     %   Detailed explanation goes here
        %     outputArg = obj.Property1 + inputArg;
        % end
    end
end

