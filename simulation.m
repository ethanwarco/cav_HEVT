classdef Vehicle
    properties
        Position double
        Speed double
        Lane int32
        IsAutonomous logical
    end
    
    methods
        function obj = Vehicle(position, speed, lane, isAutonomous)
            obj.Position = position;
            obj.Speed = speed;
            obj.Lane = lane;
            obj.IsAutonomous = isAutonomous;
        end
        
        function obj = updatePosition(obj, timeStep)
            obj.Position = obj.Position + obj.Speed * timeStep;
        end
    end
end
